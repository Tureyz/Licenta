#include "SimulationManager.h"
#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Sphere.h"
#include "../Rendering/Models/Tetrahedron.h"
#include "../Simulation/Scenario.h"
#include "../Core/Utils.hpp"
#include "../Simulation/ScenarioGenerator.h"
#include "../Rendering/Models/Cylinder.h"
#include "../Rendering/Models/Cone.h"
#include "../Collision/SeparatingAxisTheorem.h"
#include "../Benchmark/Plotter.h"

Managers::SimulationManager::SimulationManager(ModelManager *modelManager)
{
	m_objectIDCounter = 0;
	m_modelManager = modelManager;
	m_allObjects = m_modelManager->GetModelListPtr();
	m_collisionDebug = false;
	m_objectBBs = false;
	m_runningBenchmark = false;
	m_currentSimulationFrame = 0;
	m_maxSimulationFrame = 0;
	m_lastActiveMethodName = m_defaultMethodName;
}

Managers::SimulationManager::~SimulationManager()
{

}

void Managers::SimulationManager::Init()
{
	//The next line generates new scenarios every time. Comment for persistence
	//Simulation::ScenarioGenerator::ExportScenarios(Simulation::ScenarioGenerator::GenerateScenarios(5));

	ImportAllAvailableScenarios();

	m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);

	InitCollisionMethods();

	std::cout << "Current method: " << (*m_activeMethod).first.c_str() << std::endl;

	LoadScenario(m_activeScenario);
}

void Managers::SimulationManager::FixedUpdate()
{
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
			RecordLastFrameResults();
		}
		else
		{
			CurrentScenarioEnded();
		}
	}
}

void Managers::SimulationManager::Update()
{
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
		m_activeMethod->second->SetShowDebug(m_collisionDebug);
		break;
	case 'e':
		m_activeMethod++;
		if (m_activeMethod == m_collisionMethods.end())
		{
			m_activeMethod = m_collisionMethods.begin();
		}
		ResetCollisions();
		std::cout << "Current Method: " << (*m_activeMethod).first.c_str() << std::endl;
		m_activeMethod->second->SetShowDebug(m_collisionDebug);
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

	std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = (*m_activeMethod).second->TestCollision();

	for (auto pair : asd)
	{
		pair.first->SetCollisionState(COLLIDING);
		pair.second->SetCollisionState(COLLIDING);
	}
}

void Managers::SimulationManager::BenchmarkAllScenarios()
{
	m_runningBenchmark = true;
	CleanupCurrentScenario();
	m_currentScenarioIndex = 0;
	LoadScenario(m_scenarios[m_currentScenarioIndex]);
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
				file.open(Core::rawResultFolder + m_activeScenario->m_name + "_" + methodResult.first + ".txt", std::ios::out | std::ios::trunc);
			}
			else
			{
				file.open(Core::rawResultFolder + m_activeScenario->m_name + "_" + methodResult.first + ".txt", std::ios::out | std::ios::app);
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
	m_collisionMethods["BVH"]->SetIndices(&m_modelManager->m_cubeIndices);

	m_collisionMethods["Octree"] = new Collision::Octree(m_allObjects, glm::vec3(-20, -20, -20), glm::vec3(20, 20, 20));
	m_collisionMethods["Octree"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods["Octree"]->SetIndices(&m_modelManager->m_cubeIndices);
	((Collision::Octree *)m_collisionMethods["Octree"])->SetParams(5, 50);

	m_collisionMethods["Spatial-Grid"] = new Collision::SpatialGrid(m_allObjects, 10);
	m_collisionMethods["Spatial-Grid"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods["Spatial-Grid"]->SetIndices(&m_modelManager->m_cubeIndices);
	((Collision::SpatialGrid *)m_collisionMethods["Spatial-Grid"])->SetParams(glm::vec3(-20, -20, -20), glm::vec3(20, 20, 20), 10);

	m_collisionMethods["Sweep-and-Prune"] = new Collision::SweepAndPrune(m_allObjects);
	m_collisionMethods["Sweep-and-Prune"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods["Sweep-and-Prune"]->SetIndices(&m_modelManager->m_cubeIndices);

	m_collisionMethods["Spatial-Hashing"] = new Collision::SpatialHashing(m_allObjects);
	m_collisionMethods["Spatial-Hashing"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods["Spatial-Hashing"]->SetIndices(&m_modelManager->m_cubeIndices);
	((Collision::SpatialHashing *)m_collisionMethods["Spatial-Hashing"])->SetCellSize(5);

	// 	m_collisionMethods["Separating Axis Theorem"] = new Collision::SeparatingAxisTheorem(m_allObjects);
	// 	m_collisionMethods["Separating Axis Theorem"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);

		//m_activeMethod = m_collisionMethods["Sweep and Prune"];
	m_activeMethod = m_collisionMethods.find(m_defaultMethodName);
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

	for (auto it = m_collisionMethods.begin(); it != m_collisionMethods.end(); ++it)
	{
		if (!(*it).first.compare(m_lastActiveMethodName))
		{
			m_activeMethod = it;
			break;
		}
	}


	m_activeMethod->second->SetShowDebug(m_collisionDebug);
	m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);
	std::cout << "Loaded Scenario: " << m_activeScenario->m_name << std::endl;
}

void Managers::SimulationManager::CleanupCurrentScenario()
{
	m_modelManager->DeleteAllModels();

	m_lastActiveMethodName = m_activeMethod->first;
	FreeCollisionMethods();

	m_currentSimulationFrame = 0;
}

void Managers::SimulationManager::RecordLastFrameResults()
{
	std::unordered_map<std::string, PerFrameCriteria> currentFrameResults;
	for (auto method : m_collisionMethods)
	{
		method.second->Update();

		ResetCollisions();

		std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = method.second->TestCollision();

		for (auto pair : asd)
		{
			pair.first->SetCollisionState(COLLIDING);
			pair.second->SetCollisionState(COLLIDING);
		}

		PerFrameCriteria currentFrameResult;
		currentFrameResult.Element = method.second->GetLastFrameCriteria();
		currentFrameResults[method.first] = currentFrameResult;
	}
	MethodFrameResult allMethodsFrameResult;
	allMethodsFrameResult.Element = currentFrameResults;
	m_benchmarkPerFrameResults.push_back(allMethodsFrameResult);
}

void Managers::SimulationManager::CurrentScenarioEnded()
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

		Benchmark::Plotter::GeneratePlotsFromRawData();
		
		LoadScenario(m_scenarios[0]);
	}
}

void Managers::SimulationManager::SpawnManyAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects)
{
	if (typeOfObjects == Simulation::PhysicsObjectType::OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			SpawnObjectAt((Simulation::PhysicsObjectType)(rand() % Simulation::PhysicsObjectType::OBJ_NUM_TOTAL), m_objectIDCounter++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f));
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
		newObj = new Rendering::Models::Cube(Core::defaultObjectColor, m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_TETRAHEDRON:
		newObj = new Rendering::Models::Tetrahedron(Core::defaultObjectColor, m_modelManager, this);
		break;

	case Simulation::PhysicsObjectType::OBJ_SPHERE:
		newObj = new Rendering::Models::Sphere(Core::defaultObjectColor, m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_CYLINDER:
		newObj = new Rendering::Models::Cylinder(Core::defaultObjectColor, m_modelManager, this);
		break;
	default:
		newObj = new Rendering::Models::Cone(Core::defaultObjectColor, m_modelManager, this);
		break;
	}

	newObj->SetID(ID);
	newObj->Create();
	newObj->ScaleAbsolute(scale);
	newObj->RotateAbsolute(rotation, rotationAngle);
	newObj->TranslateAbsolute(position);
	ObjectAdded(newObj);

	m_modelManager->RegisterObject(ID, newObj);

	return newObj;
}
