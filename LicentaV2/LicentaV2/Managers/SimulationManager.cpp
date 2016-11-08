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
#include "../Collision/SphereToSphereTest.h"

#include <algorithm>

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
	m_simulationDebug = false;
}

Managers::SimulationManager::~SimulationManager()
{

}

void Managers::SimulationManager::Init()
{
	//The next line generates new scenarios every time. Comment for persistence
	Simulation::ScenarioGenerator::ExportScenarios(Simulation::ScenarioGenerator::GenerateScenarios(5));

	ImportAllAvailableScenarios();

	m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);

	InitCollisionMethods();

	DisplayHelp();
	std::wcout << "Current Collision Method: " << (*m_activeMethod).first.c_str() << std::endl;

	LoadScenario(m_activeScenario);

	m_resultManager.Init(Core::SCENARIO_CLASSES, Core::OBJECT_INCREMENT, Core::FRAMES_NUM, Core::MAX_NUMBER_OBJECTS);
}

void Managers::SimulationManager::FixedUpdate()
{
	m_currentSimulationFrame++;

	if (!m_runningBenchmark)
	{
		(*m_activeMethod).second->Update();
		TestCollision();

		if (m_currentSimulationFrame > m_maxSimulationFrame && Core::REPLAY_SCENARIO)
		{
			CleanupCurrentScenario();
			m_currentScenarioIndex.second++;
			if (m_currentScenarioIndex.second == m_scenarios[m_currentScenarioIndex.first].size())
			{
				m_currentScenarioIndex.second = 0;
			}

			LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
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
	if (m_runningBenchmark)
		return;

	switch (key)
	{
	case 'q':
		if (m_activeMethod == m_collisionMethods.begin())
		{
			m_activeMethod = m_collisionMethods.end();
		}
		m_activeMethod--;
		ResetCollisions();
		std::wcout << "Current Collision Method: " << (*m_activeMethod).first.c_str() << std::endl;
		m_activeMethod->second->SetShowDebug(m_collisionDebug);
		break;
	case 'e':
		m_activeMethod++;
		if (m_activeMethod == m_collisionMethods.end())
		{
			m_activeMethod = m_collisionMethods.begin();
		}
		ResetCollisions();
		std::wcout << "Current Collision Method: " << (*m_activeMethod).first.c_str() << std::endl;
		m_activeMethod->second->SetShowDebug(m_collisionDebug);
		break;
	case 'r':
		m_collisionDebug = !m_collisionDebug;
		//(*m_activeMethod).second->SetShowDebug(m_collisionDebug);
		for (auto method : m_collisionMethods)
		{
			method.second->SetShowDebug(m_collisionDebug);
		}
		std::wcout << "Collision Debug " << (m_collisionDebug ? "ON" : "OFF") << std::endl;
		break;
	case 't':
		m_objectBBs = !m_objectBBs;
		m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);
		std::wcout << "Bounding Volumes " << (m_objectBBs ? "ON" : "OFF") << std::endl;
		break;
	case 'y':
		m_simulationDebug = !m_simulationDebug;
		std::wcout << "Simulation Debug " << (m_simulationDebug ? "ON" : "OFF") << std::endl;

		break;
	case 'b':
		BenchmarkAllScenarios();
		break;
	case 'x':
		CleanupCurrentScenario();
		m_currentScenarioIndex.first++;
		if (m_currentScenarioIndex.first >= m_scenarios.size())
		{
			m_currentScenarioIndex.first = 0;
		}
		m_currentScenarioIndex.second = 0;
		LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
		break;
	case 'z':
		CleanupCurrentScenario();
		m_currentScenarioIndex.first--;
		if (m_currentScenarioIndex.first <= -1)
		{
			m_currentScenarioIndex.first = (int) (m_scenarios.size() - 1);
		}
		m_currentScenarioIndex.second = 0;
		LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
		break;
	case 'h':
		DisplayHelp();
		break;
	default:
		break;
	}
}

void Managers::SimulationManager::KeyReleased(unsigned char key)
{

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

	m_currentCollisionPairs = (*m_activeMethod).second->TestCollision();

	for (auto pair : m_currentCollisionPairs)
	{
		pair.first->SetCollisionState(COLLIDING);
		pair.second->SetCollisionState(COLLIDING);
	}
}

void Managers::SimulationManager::BenchmarkAllScenarios()
{

	m_collisionDebug = false;
	for (auto method : m_collisionMethods)
	{
		method.second->SetShowDebug(m_collisionDebug);
	}

	m_objectBBs = false;
	m_modelManager->SetBoundingBoxesVisibile(m_objectBBs);

	m_runningBenchmark = true;
	CleanupCurrentScenario();
	m_currentScenarioIndex = std::make_pair(0, 0);
	LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
	m_currentSimulationFrame = 0;
	m_benchmarkStartTime = std::chrono::high_resolution_clock::now();
	std::wcout << "Started benchmark" << std::endl;
	std::wcout << "Progress:   0%";
}

void Managers::SimulationManager::ImportAllAvailableScenarios()
{
	for (auto fileName : Core::Utils::GetAllFilesInFolder(L"SavedScenarios"))
	{
		//std::wcout << fileName << std::endl;

		m_scenarios.push_back(std::vector<Simulation::Scenario *>());

		for (int i = Core::OBJECT_INCREMENT; ; i += Core::OBJECT_INCREMENT)
		{
			Simulation::Scenario *scen = new Simulation::Scenario();
			scen->LoadFromFile(L"SavedScenarios/" + fileName, i);
			if (scen->GetObjectDescriptions().size() != i)
			{
				break;
			}
			m_scenarios[m_scenarios.size() - 1].push_back(scen);
		}
	}

	if (!m_scenarios.empty())
	{
		m_activeScenario = m_scenarios[0][0];
		m_currentScenarioIndex = std::make_pair(0, 0);
	}
	else
	{
		m_activeScenario = NULL;
		m_currentScenarioIndex = std::make_pair(-1, -1);
	}
}

void Managers::SimulationManager::InitCollisionMethods()
{
	m_collisionMethods[Core::METHOD_NONE] = new Collision::DummyMethod(m_allObjects);

// 	m_collisionMethods[Core::METHOD_BVH] = new Collision::BVH(m_allObjects);
// 	m_collisionMethods[Core::METHOD_BVH]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
// 	m_collisionMethods[Core::METHOD_BVH]->SetIndices(&m_modelManager->m_cubeIndices);
// 
// 	m_collisionMethods[Core::METHOD_OCTREE] = new Collision::Octree(m_allObjects, glm::vec3(-20, -20, -20)/2.f, glm::vec3(20, 20, 20)/2.f);
// 	m_collisionMethods[Core::METHOD_OCTREE]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
// 	m_collisionMethods[Core::METHOD_OCTREE]->SetIndices(&m_modelManager->m_lineCubeIndices);
// 	((Collision::Octree *)m_collisionMethods[Core::METHOD_OCTREE])->SetParams(5, 50);
// 
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID] = new Collision::SpatialGrid(m_allObjects, 10);
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID]->SetIndices(&m_modelManager->m_lineCubeIndices);
// 	((Collision::SpatialGrid *)m_collisionMethods[Core::METHOD_SPATIAL_GRID])->SetParams(glm::vec3(-20, -20, -20)/2.f, glm::vec3(20, 20, 20)/2.f, 4);
// 
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID_OPTIMIZED] = new Collision::SpatialGrid(m_allObjects, 10);
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID_OPTIMIZED]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
// 	m_collisionMethods[Core::METHOD_SPATIAL_GRID_OPTIMIZED]->SetIndices(&m_modelManager->m_lineCubeIndices);
// 	((Collision::SpatialGrid *)m_collisionMethods[Core::METHOD_SPATIAL_GRID_OPTIMIZED])->SetParams(glm::vec3(-20, -20, -20), glm::vec3(20, 20, 20), 10);
// 
	m_collisionMethods[Core::METHOD_SAP] = new Collision::SweepAndPrune(m_allObjects);
	m_collisionMethods[Core::METHOD_SAP]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods[Core::METHOD_SAP]->SetIndices(&m_modelManager->m_cubeIndices);
// 
// 	m_collisionMethods[Core::METHOD_SPATIAL_HASHING] = new Collision::SpatialHashing(m_allObjects);
// 	m_collisionMethods[Core::METHOD_SPATIAL_HASHING]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
// 	m_collisionMethods[Core::METHOD_SPATIAL_HASHING]->SetIndices(&m_modelManager->m_cubeIndices);
// 	((Collision::SpatialHashing *)m_collisionMethods[Core::METHOD_SPATIAL_HASHING])->SetCellSize(5);


	//m_collisionMethods[Core::METHOD_S2S] = new Collision::SphereToSphereTest(m_allObjects);

	// 	m_collisionMethods["Separating Axis Theorem"] = new Collision::SeparatingAxisTheorem(m_allObjects);
	// 	m_collisionMethods["Separating Axis Theorem"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);

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
		std::wcout << "NULL SCEN\n";
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

	if (m_simulationDebug)
	{		
		std::wcout << "Loaded Scenario: " << m_activeScenario->m_name << " - " << m_activeScenario->GetObjectDescriptions().size() << " objects" << std::endl;
	}
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
	// 	std::vector<std::wstring > keys(m_collisionMethods.size());
	// 	std::vector <Collision::ICollisionMethod*> values(m_collisionMethods.size());
	// 
	// 	transform(m_collisionMethods.begin(), m_collisionMethods.end(), keys.begin(), [](auto pair) {return pair.first; });
	// 	transform(m_collisionMethods.begin(), m_collisionMethods.end(), values.begin(), [](auto pair) {return pair.second; });

		//#pragma omp parallel for
		//for (int i = 0; i < m_collisionMethods.size(); ++i)
	for (auto method : m_collisionMethods)
	{
		auto methodName = method.first;
		auto methodPointer = method.second;

		//		auto methodName = keys[i];
		//		auto methodPointer = values[i];

		methodPointer->Update();

		ResetCollisions();

		std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = methodPointer->TestCollision();

		for (auto criterion : methodPointer->GetLastFrameCriteria())
		{
			m_resultManager.RecordCriterion(methodName, criterion.first, criterion.second);
		}

		// 		for (auto pair : asd)
		// 		{
		// 			pair.first->SetCollisionState(COLLIDING);
		// 			pair.second->SetCollisionState(COLLIDING);
		// 		}

	}

	m_resultManager.FrameEnded();

}

std::wstring  HumanReadableTime(size_t seconds)
{
	size_t minutes = seconds / 60;
	size_t hours = minutes / 60;

	return std::wstring (std::to_wstring(hours) + L" hours, " + std::to_wstring(minutes % 60) + L" minutes, " + std::to_wstring(seconds % 60) + L" seconds");
}

void Managers::SimulationManager::CurrentScenarioEnded()
{
	DisplayPercentComplete();

	CleanupCurrentScenario();

	if (m_currentScenarioIndex.second < m_scenarios[m_currentScenarioIndex.first].size() - 1)
	{
		m_currentScenarioIndex.second++;
		m_resultManager.ScenarioEnded();

		//LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
	}
	else
	{
		if (m_currentScenarioIndex.first < m_scenarios.size() - 1)
		{
			m_currentScenarioIndex.first++;
			m_currentScenarioIndex.second = 0;

			m_resultManager.ScenarioClassEnded();

			//LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
		}
		else
		{
			m_currentScenarioIndex.second++;
			DisplayPercentComplete();
			std::wcout << std::endl;
			m_currentScenarioIndex.first = 0;
			m_currentScenarioIndex.second = 0;

			m_runningBenchmark = false;

			m_resultManager.DumpToDisk();
			std::wcout << "Finished Benchmark" << std::endl;
			auto end = std::chrono::high_resolution_clock::now();

			auto timeSpent = std::chrono::duration_cast<std::chrono::seconds>(end - m_benchmarkStartTime).count();

			DEVMODE lpDevMode;
			memset(&lpDevMode, 0, sizeof(DEVMODE));
			lpDevMode.dmSize = sizeof(DEVMODE);
			lpDevMode.dmDriverExtra = 0;
			EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &lpDevMode);

			int refreshRate = lpDevMode.dmDisplayFrequency;

			std::wcout << "Benchmark Info:" << std::endl;
			std::wcout << "Number of scenario classes: " << m_scenarios.size() << std::endl;
			std::wcout << "Number of frames per scenario: " << m_activeScenario->m_numberOfFrames << " frames" << std::endl;
			std::wcout << "Number of collision methods: " << m_collisionMethods.size() << std::endl;
			std::wcout << "Each scenario class scaled the number of objects from " << Core::OBJECT_INCREMENT << " to " << (m_scenarios[0].size()) * Core::OBJECT_INCREMENT << " in increments of " << Core::OBJECT_INCREMENT << std::endl;
			std::wcout << "Best possible time, based on number of required frames and monitor refresh rate (" << refreshRate << "Hz): " << HumanReadableTime(Core::SCENARIO_CLASSES * Core::FRAMES_NUM * (Core::MAX_NUMBER_OBJECTS / Core::OBJECT_INCREMENT) / refreshRate) << std::endl;
			std::wcout << "Actual time spent: " << HumanReadableTime(timeSpent) << std::endl;
		}
	}

	LoadScenario(m_scenarios[m_currentScenarioIndex.first][m_currentScenarioIndex.second]);
}

void Managers::SimulationManager::DisplayPercentComplete()
{
	auto percentComplete = PercentFinished();
	for (int i = 0; i < percentComplete.size(); ++i)
	{
		std::wcout << "\b";
	}
	std::wcout << percentComplete;
}

void Managers::SimulationManager::DisplayHelp()
{
	std::wcout << L"------------------------------------------------" << std::endl;
	std::wcout << L"Available Controls:" << std::endl;
	std::wcout << L"H - display this help message" << std::endl;
	std::wcout << L"WASD - camera movement" << std::endl;
	std::wcout << L"Mouse drag - camera rotation" << std::endl;
	std::wcout << L"Q / E - previous / next active collision method" << std::endl;
	std::wcout << L"R - toggle collision method debug draw" << std::endl;
	std::wcout << L"T - toggle bounding volume debug draw" << std::endl;
	std::wcout << L"Z / X - previous / next scenario class" << std::endl;
	std::wcout << L"Y - toggle simulation debug messages" << std::endl;
	std::wcout << L"B - start benchmark" << std::endl;
	std::wcout << L"------------------------------------------------" << std::endl;
}

std::wstring  Managers::SimulationManager::PercentFinished()
{
	float total = Core::SCENARIO_CLASSES * Core::MAX_NUMBER_OBJECTS / Core::OBJECT_INCREMENT;

	float progress = (m_currentScenarioIndex.first * Core::MAX_NUMBER_OBJECTS / Core::OBJECT_INCREMENT) + m_currentScenarioIndex.second;

	float percent = (progress / total) * 100;

	return std::wstring ((percent < 10 ? L" " : L"") + std::to_wstring((int)percent) + L"%");
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
		newObj = new Rendering::Models::Cube(Core::DEFAULT_OBJECT_COLOR, m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_TETRAHEDRON:
		newObj = new Rendering::Models::Tetrahedron(Core::DEFAULT_OBJECT_COLOR, m_modelManager, this);
		break;

	case Simulation::PhysicsObjectType::OBJ_SPHERE:
		newObj = new Rendering::Models::Sphere(Core::DEFAULT_OBJECT_COLOR, m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_CYLINDER:
		newObj = new Rendering::Models::Cylinder(Core::DEFAULT_OBJECT_COLOR, m_modelManager, this);
		break;
	default:
		newObj = new Rendering::Models::Cone(Core::DEFAULT_OBJECT_COLOR, m_modelManager, this);
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
