#include "SimulationManager.h"
#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Sphere.h"
#include "../Rendering/Models/Tetrahedron.h"
#include "../Simulation/Scenario.h"
#include "../Core/Utils.hpp"
#include "../Simulation/ScenarioGenerator.h"

Managers::SimulationManager::SimulationManager(ModelManager *modelManager)
{
	m_objectIDCounter = 0;
	m_time = m_timeBase = 0;
	m_modelManager = modelManager;
	m_allObjects = m_modelManager->GetModelListPtr();
	m_collisionDebug = false;
	m_objectBBs = false;
	m_runningBenchmark = false;
	m_currentSimulationFrame = 0;
	m_maxSimulationFrame = 0;
}

Managers::SimulationManager::~SimulationManager()
{

}

void Managers::SimulationManager::Init()
{
// 	SpawnManyAround(glm::vec3(0.f, 0.f, 0.f), 7.f, 5, Simulation::PhysicsObjectType::OBJ_RANDOM);
// 	m_modelManager->SetBoundingBoxesVisibile(false);
// 
// 	for (int i = 0; i < m_modelManager->GetModelListPtr()->size(); ++i)
// 	{
// 		(*m_modelManager->GetModelListPtr())[i]->SetRotationStep(glm::vec3(1.001f));
// 		(*m_modelManager->GetModelListPtr())[i]->SetRotationAngleStep(0.01f);
// 	}
	
	Simulation::ScenarioGenerator::ExportScenarios(Simulation::ScenarioGenerator::GenerateScenarios(5));
	//Simulation::ScenarioGenerator::GenerateScenarios(5);
// 	Simulation::Scenario *scen = new Simulation::Scenario();
// 	//scen.LoadFromObjects(*m_allObjects, "Test Scenario");
// 	//scen.SaveToFile("SavedScenarios/test.txt");
// 	scen->LoadFromFile("SavedScenarios/test.txt");
// 	SpawnObjectsFromScenario(scen);


	ImportAllAvailableScenarios();

// 	m_scenarios.push_back(scen);
// 	m_activeScenario = scen;
// 	m_currentScenarioIndex = 0;

	m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);
	
	InitCollisionMethods();

	std::cout << "Current method: " << (*m_activeMethod).first.c_str() << std::endl;


	LoadScenario(m_activeScenario);
}

void Managers::SimulationManager::Update()
{
	// Collision checks 30 times per second
	m_time = glutGet(GLUT_ELAPSED_TIME);
	if (m_time - m_timeBase > 1000.f / 30.f)
	{
		m_timeBase = m_time;
		//std::cout << "TICK\n";
		m_currentSimulationFrame++;

		if (!m_runningBenchmark)
		{
			(*m_activeMethod).second->Update();
			TestCollision();

			if (m_currentSimulationFrame > m_maxSimulationFrame)
			{
				CleanupCurrentScenario();
				LoadScenario(m_activeScenario);

			}
		}
		else
		{
			if (m_currentSimulationFrame <= m_maxSimulationFrame)
			{
				std::unordered_map<std::string, PerFrameCriteria> currentFrameResults;
				for (auto method : m_collisionMethods)
				{
					method.second->Update();

					ResetCollisions();

					std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = method.second->TestCollision();

					for (int i = 0; i < asd.size(); ++i)
					{
						asd[i].first->SetCollisionState(COLLIDING);
						asd[i].second->SetCollisionState(COLLIDING);
					}

					PerFrameCriteria currentFrameResult;
					currentFrameResult.Element = method.second->GetLastFrameCriteria();
					currentFrameResults[method.first] = currentFrameResult;
				}
				MethodFrameResult allMethodsFrameResult;
				allMethodsFrameResult.Element = currentFrameResults;
				m_benchmarkPerFrameResults.push_back(allMethodsFrameResult);
				
			} 
			else
			{
				CleanupCurrentScenario();
				ExportCurrentScenarioResults();
				m_currentScenarioIndex++;

				if (m_currentScenarioIndex < m_scenarios.size())
				{
					LoadScenario(m_scenarios[m_currentScenarioIndex]);					
				}
				else
				{
					m_runningBenchmark = false;
					std::cout << "Finished Benchmark" << std::endl;					
					LoadScenario(m_scenarios[0]);
				}
			}
		}
	}
}

void Managers::SimulationManager::Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	(*m_activeMethod).second->DrawDebug(projectionMatrix, viewMatrix);
}

void Managers::SimulationManager::ObjectMoved(IPhysicsObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectMoved(object);
	}
}

void Managers::SimulationManager::ObjectAdded(IPhysicsObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectAdded(object);
	}
}

void Managers::SimulationManager::ObjectRemoved(IPhysicsObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectRemoved(object);
	}
}

void Managers::SimulationManager::KeyPressed(unsigned char key)
{
	switch (key)
	{
	case 'q':
		if (m_activeMethod == m_collisionMethods.begin())
		{
			m_activeMethod = m_collisionMethods.end();
		}
		m_activeMethod--;
		ResetCollisions();
		std::cout << "Current Method: " << (*m_activeMethod).first.c_str() << std::endl;
		break;
	case 'e':
		m_activeMethod++;
		if (m_activeMethod == m_collisionMethods.end())
		{
			m_activeMethod = m_collisionMethods.begin();
		}
		ResetCollisions();
		std::cout << "Current Method: " << (*m_activeMethod).first.c_str() << std::endl;
		break;
	case 'r':
		m_collisionDebug = !m_collisionDebug;
		//(*m_activeMethod).second->SetShowDebug(m_collisionDebug);
		for (auto method : m_collisionMethods)
		{
			method.second->SetShowDebug(m_collisionDebug);
		}
		std::cout << "Collision Debug " << (m_collisionDebug ? "ON" : "OFF") << std::endl;
		break;
	case 't':
		m_objectBBs = !m_objectBBs;
		m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);
		break;
	case 'b':
		BenchmarkAllScenarios();
		break;
	case 'x':
		CleanupCurrentScenario();
		m_currentScenarioIndex++;
		if (m_currentScenarioIndex >= m_scenarios.size())
		{
			m_currentScenarioIndex = 0;
		}
		LoadScenario(m_scenarios[m_currentScenarioIndex]);
		break;
	case 'z':
		CleanupCurrentScenario();
		m_currentScenarioIndex--;
		if (m_currentScenarioIndex <= -1)
		{
			m_currentScenarioIndex = m_scenarios.size() - 1;
		}
		LoadScenario(m_scenarios[m_currentScenarioIndex]);
		break;
	default:
		break;
	}
}

void Managers::SimulationManager::MousePressed(int button, int state, int x, int y)
{

}

void Managers::SimulationManager::MouseMove(int x, int y, int width, int height)
{

}

void Managers::SimulationManager::Draw()
{

}

void Managers::SimulationManager::TestCollision()
{

	// one to many

	// 	auto asd = m_test->TestCollision(m_physicsModelList[0]);
	// 	m_physicsModelList[0]->SetCollisionState(ACTIVE);
	// 	for (auto obj : asd) {
	// 		obj->SetCollisionState(COLLIDING);
	// 	}

	//many to many

	ResetCollisions();

	std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = (*m_activeMethod).second->TestCollision();
	
	for (int i = 0; i < asd.size(); ++i)
	{
		asd[i].first->SetCollisionState(COLLIDING);
		asd[i].second->SetCollisionState(COLLIDING);
	}
}

void Managers::SimulationManager::BenchmarkAllScenarios()
{
	m_runningBenchmark = true;
	CleanupCurrentScenario();
	LoadScenario(m_scenarios[0]);
	m_currentSimulationFrame = 0;
	std::cout << "Starting benchmark..." << std::endl;
}

void Managers::SimulationManager::ExportCurrentScenarioResults()
{
	for (int i = 0; i < m_benchmarkPerFrameResults.size(); ++i)
	{
		auto frameResult = m_benchmarkPerFrameResults[i];		
		for (auto methodResult : frameResult.Element)
		{
			std::ofstream file;
			if (i == 0)
			{
				file.open("BenchmarkResults/" + m_activeScenario->m_name + "_" + methodResult.first + ".txt", std::ios::out | std::ios::trunc);
			}
			else
			{
				file.open("BenchmarkResults/" + m_activeScenario->m_name + "_" + methodResult.first + ".txt", std::ios::out | std::ios::app);
			}

			if (!file.is_open())
			{
				std::cout << "ERROR opening file\n";
			}

			auto criteria = methodResult.second;

			if (i == 0)
			{
				file << "[Frame] ";
				for (auto criterion : criteria.Element)
				{
					file << "[" << criterion.first << "] ";
				}

				file << std::endl;
			}
			file << i << " ";

			for (auto criterion : criteria.Element)
			{
				file << criterion.second << " ";
			}
			file << std::endl;

			file.close();
		}
	}

	m_benchmarkPerFrameResults.clear();
}

void Managers::SimulationManager::ImportAllAvailableScenarios()
{
	for (auto fileName : Core::Utils::GetAllFilesInFolder("SavedScenarios"))
	{
		//std::cout << fileName << std::endl;

		Simulation::Scenario *scen = new Simulation::Scenario();
		scen->LoadFromFile("SavedScenarios/" + fileName);
		m_scenarios.push_back(scen);
	}

	if (!m_scenarios.empty())
	{
		m_activeScenario = m_scenarios[0];
		m_currentScenarioIndex = 0;
	}
	else 
	{
		m_activeScenario = NULL;
		m_currentScenarioIndex = -1;
	}
}

void Managers::SimulationManager::InitCollisionMethods()
{
	m_collisionMethods["None"] = new Collision::DummyMethod(m_allObjects);

	m_collisionMethods["BVH"] = new Collision::BVH(m_allObjects);
	m_collisionMethods["BVH"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);

	m_collisionMethods["Octree"] = new Collision::Octree(m_allObjects, glm::vec3(-20, -20, -20), glm::vec3(20, 20, 20));
	m_collisionMethods["Octree"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::Octree *)m_collisionMethods["Octree"])->SetParams(5, 50);

	m_collisionMethods["Spatial Grid"] = new Collision::SpatialGrid(m_allObjects, 10);
	m_collisionMethods["Spatial Grid"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::SpatialGrid *)m_collisionMethods["Spatial Grid"])->SetParams(glm::vec3(-20, -20, -20), glm::vec3(20, 20, 20), 10);

	m_collisionMethods["Sweep and Prune"] = new Collision::SweepAndPrune(m_allObjects);
	m_collisionMethods["Sweep and Prune"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);

	m_collisionMethods["Spatial Hashing"] = new Collision::SpatialHashing(m_allObjects);
	m_collisionMethods["Spatial Hashing"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::SpatialHashing *)m_collisionMethods["Spatial Hashing"])->SetCellSize(5);

	//m_activeMethod = m_collisionMethods["Sweep and Prune"];
	m_activeMethod = m_collisionMethods.find("None");
	//(*m_activeMethod).second->SetShowDebug(true);
}

void Managers::SimulationManager::FreeCollisionMethods()
{
	for (auto method : m_collisionMethods)
	{
		delete(method.second);
	}
}

void Managers::SimulationManager::ResetCollisions()
{
	for (auto obj : *m_allObjects)
	{
		obj->SetCollisionState(DEFAULT);
	}
}

void Managers::SimulationManager::LoadScenario(Simulation::Scenario *scenario)
{
	if (!scenario)
	{
		std::cout << "NULL SCEN\n";
		return;
	}
	m_activeScenario = scenario;
	m_maxSimulationFrame = m_activeScenario->m_numberOfFrames;	

	InitCollisionMethods();
	SpawnObjectsFromScenario(m_activeScenario);
	std::cout << "Loaded Scenario: " << m_activeScenario->m_name << std::endl;
}

void Managers::SimulationManager::CleanupCurrentScenario()
{	
	m_modelManager->DeleteAllModels();

	FreeCollisionMethods();

	m_currentSimulationFrame = 0;
}

void Managers::SimulationManager::SpawnManyAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects)
{
	if (typeOfObjects == Simulation::PhysicsObjectType::OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			SpawnObjectAt((Simulation::PhysicsObjectType)(rand() % 3), m_objectIDCounter++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f));
		}
	}
	else
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			SpawnObjectAt(typeOfObjects, m_objectIDCounter++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f));
		}
	}
}

void Managers::SimulationManager::SpawnObjectsFromScenario(const Simulation::Scenario *scenario)
{
	for (auto desc : scenario->GetObjectDescriptions())
	{
		IPhysicsObject *obj = SpawnObjectAt(desc.m_objectType, desc.m_ID, desc.m_initialPosition, desc.m_initialRotation, desc.m_initialRotationAngle, desc.m_initialScale);
		obj->SetTranslationStep(desc.m_translationStep);
		obj->SetScaleStep(desc.m_scaleStep);
		obj->SetRotationStep(desc.m_rotationStep);
		obj->SetRotationAngleStep(desc.m_rotationAngleStep);
	}
}

Rendering::IPhysicsObject* Managers::SimulationManager::SpawnObjectAt(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale)
{
	Rendering::Models::Model *newObj;

	switch (objectType)
	{
	case Simulation::PhysicsObjectType::OBJ_CUBE:
		newObj = new Rendering::Models::Cube(m_defaultObjectColor, m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_TETRAHEDRON:
		newObj = new Rendering::Models::Tetrahedron(m_defaultObjectColor, m_modelManager, this);
		break;
	default:
		newObj = new Rendering::Models::Sphere(m_defaultObjectColor, m_modelManager, this);
		break;
	}

	newObj->SetID(ID);
	newObj->Create();
	newObj->ScaleAbsolute(scale);
	newObj->RotateAbsolute(rotation, rotationAngle);
	newObj->TranslateAbsolute(position);
	ObjectAdded(newObj);

	m_modelManager->SetModel(ID, newObj);

	return newObj;
}
