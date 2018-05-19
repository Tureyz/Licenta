#include "BachelorSimulationManager.h"

#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Sphere.h"
#include "../Rendering/Models/Tetrahedron.h"
#include "../Simulation/Scenario.h"
#include "../Core/Utils.hpp"
#include "../Simulation/ScenarioGenerator.h"
#include "../Collision/SeparatingAxisTheorem.h"
#include "../Collision/SphereToSphereTest.h"

#include "../Dependencies/glm/gtx/rotate_vector.hpp"

#include <algorithm>

#define PI 3.14159f

Managers::BachelorSimulationManager::BachelorSimulationManager(ModelManager *modelManager)
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
	m_firstAvailableID = 0;
}

Managers::BachelorSimulationManager::~BachelorSimulationManager()
{

}

void Managers::BachelorSimulationManager::Init()
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

void Managers::BachelorSimulationManager::FixedUpdate()
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

void Managers::BachelorSimulationManager::Update()
{
	for (auto obj : m_removeQueue)
	{
		m_modelManager->DeleteModel(obj->GetID());
		ObjectRemoved(obj);
	}

	m_removeQueue.clear();

	for (auto desc : m_addQueue)
	{
		SceneObject *obj = SpawnObjectAt(desc.m_objectType, desc.m_ID, desc.m_initialPosition, desc.m_initialRotation, desc.m_initialRotationAngle, desc.m_initialScale);
		obj->SetTranslationStep(desc.m_translationStep);
		obj->SetScaleStep(desc.m_scaleStep);
		obj->SetRotationStep(desc.m_rotationStep);
		obj->SetRotationAngleStep(desc.m_rotationAngleStep);

		ObjectAdded(obj);
	}

	m_addQueue.clear();
}

void Managers::BachelorSimulationManager::Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	(*m_activeMethod).second->DrawDebug(projectionMatrix, viewMatrix);
}

void Managers::BachelorSimulationManager::ObjectMoved(SceneObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectMoved(object);
	}
}

void Managers::BachelorSimulationManager::ObjectAdded(SceneObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectAdded(object);
	}
}

void Managers::BachelorSimulationManager::ObjectRemoved(SceneObject *object)
{
	for (auto method : m_collisionMethods)
	{
		method.second->ObjectRemoved(object);
	}
}

void Managers::BachelorSimulationManager::KeyPressed(unsigned char key)
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
		DebugBreakAll();
		//BenchmarkAllScenarios();
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
			m_currentScenarioIndex.first = (int)(m_scenarios.size() - 1);
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

void Managers::BachelorSimulationManager::KeyReleased(unsigned char key)
{

}

void Managers::BachelorSimulationManager::MousePressed(int button, int state, int x, int y)
{

}

void Managers::BachelorSimulationManager::MouseMove(int x, int y, int width, int height)
{

}

void Managers::BachelorSimulationManager::Draw()
{

}

void Managers::BachelorSimulationManager::TestCollision()
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

void Managers::BachelorSimulationManager::BenchmarkAllScenarios()
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

void Managers::BachelorSimulationManager::ImportAllAvailableScenarios()
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

void Managers::BachelorSimulationManager::CreateWorldBounds()
{
	m_worldBounds = std::make_pair(glm::vec3(-50, -50, -50), glm::vec3(50, 50, 50));
}

void Managers::BachelorSimulationManager::InitCollisionMethods()
{
	m_collisionMethods[Core::METHOD_NONE] = new Collision::DummyMethod(m_allObjects);

	// 	m_collisionMethods[Core::METHOD_BVH] = new Collision::BVH(m_allObjects);
	// 	m_collisionMethods[Core::METHOD_BVH]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	// 	m_collisionMethods[Core::METHOD_BVH]->SetIndices(&m_modelManager->m_cubeIndices);
	// 
	m_collisionMethods[Core::METHOD_OCTREE] = new Collision::Octree(m_allObjects, glm::vec3(-50, -50, -50), glm::vec3(50, 50, 50));
	m_collisionMethods[Core::METHOD_OCTREE]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	m_collisionMethods[Core::METHOD_OCTREE]->SetIndices(&m_modelManager->m_lineCubeIndices);
	((Collision::Octree *)m_collisionMethods[Core::METHOD_OCTREE])->SetParams(5, 10);
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
	// 	m_collisionMethods[Core::METHOD_SAP] = new Collision::SweepAndPrune(m_allObjects);
	// 	m_collisionMethods[Core::METHOD_SAP]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	// 	m_collisionMethods[Core::METHOD_SAP]->SetIndices(&m_modelManager->m_cubeIndices);
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

void Managers::BachelorSimulationManager::FreeCollisionMethods()
{
	for (auto method : m_collisionMethods)
	{
		delete(method.second);
	}
}

void Managers::BachelorSimulationManager::BreakObject(Rendering::SceneObject *obj, glm::vec3 impactForce)
{
	if (obj->GetObjectType() != 1)
		return;

	//std::wcout << L"BREAK " << glm::length(impactForce) << " " << obj->GetSphereRadius() << " " << obj->GetMass() << std::endl;
	float parentRadius = obj->GetSphereRadius();
	float childRadius = parentRadius * m_childSphereCoef;

	glm::vec3 parentScale = obj->GetScale();
	glm::vec3 childScale = parentScale * m_childSphereCoef;

	glm::vec3 parentPos = obj->GetPosition();

	float offset = childRadius;

	std::vector<glm::vec3> relativeSpawnPositions = { glm::vec3(-1, 0, 0), glm::vec3(1, 0, 0), glm::vec3(0, -1, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, -1), glm::vec3(0, 0, 1) };

	glm::vec3 rotationAxis = glm::normalize(glm::vec3(std::rand() % 360, std::rand() % 360, std::rand() % 360));
	float rotationAngles = static_cast<float>(std::rand() % 360);

	float childDensity = obj->GetMass() / ((8 * PI) * childRadius * childRadius * childRadius);

	for (auto relVec : relativeSpawnPositions)
	{
		auto asd = glm::normalize(glm::rotate(relVec, rotationAngles, rotationAxis));

		glm::vec3 childPos = parentPos + asd * offset;
		auto def = Simulation::ScenarioGenerator::CreateDef(Simulation::OBJ_SPHERE, m_firstAvailableID++, childPos, Core::Utils::Random01(), (float)(std::rand() % 360), childScale);
		def.m_translationStep = glm::length(impactForce) * glm::normalize(childPos - parentPos);
		def.m_rotationStep = obj->GetRotationStep();
		def.m_rotationAngleStep = glm::length(obj->GetRotationStep());
		def.m_density = childDensity;
		m_addQueue.push_back(def);
	}


	obj->SetBroken(true);
	m_removeQueue.push_back(obj);
}

void Managers::BachelorSimulationManager::DebugBreakAll()
{
	for (auto obj : *m_allObjects)
	{
		BreakObject(obj, glm::normalize(obj->GetPosition()) * 0.005f);
	}
}

void Managers::BachelorSimulationManager::ResetCollisions()
{
	for (auto obj : *m_allObjects)
	{
		obj->SetCollisionState(DEFAULT);
	}
}

void Managers::BachelorSimulationManager::LoadScenario(Simulation::Scenario *scenario)
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

void Managers::BachelorSimulationManager::CleanupCurrentScenario()
{
	m_modelManager->DeleteAllModels();

	m_lastActiveMethodName = m_activeMethod->first;
	FreeCollisionMethods();

	m_currentSimulationFrame = 0;
}

void Managers::BachelorSimulationManager::RecordLastFrameResults()
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

		std::unordered_set<std::pair<SceneObject *, SceneObject *>> asd = methodPointer->TestCollision();

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

	return std::wstring(std::to_wstring(hours) + L" hours, " + std::to_wstring(minutes % 60) + L" minutes, " + std::to_wstring(seconds % 60) + L" seconds");
}

void Managers::BachelorSimulationManager::CurrentScenarioEnded()
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

void Managers::BachelorSimulationManager::DisplayPercentComplete()
{
	auto percentComplete = PercentFinished();
	for (int i = 0; i < percentComplete.size(); ++i)
	{
		std::wcout << "\b";
	}
	std::wcout << percentComplete;
}

void Managers::BachelorSimulationManager::DisplayHelp()
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

std::wstring  Managers::BachelorSimulationManager::PercentFinished()
{
	float total = Core::SCENARIO_CLASSES * Core::MAX_NUMBER_OBJECTS / Core::OBJECT_INCREMENT;

	float progress = static_cast<float>((m_currentScenarioIndex.first * Core::MAX_NUMBER_OBJECTS / Core::OBJECT_INCREMENT) + m_currentScenarioIndex.second);

	float percent = (progress / total) * 100;

	return std::wstring((percent < 10 ? L" " : L"") + std::to_wstring((int)percent) + L"%");
}

void Managers::BachelorSimulationManager::SpawnManyAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects)
{
	if (typeOfObjects == Simulation::PhysicsObjectType::OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			SpawnObjectAt((Simulation::PhysicsObjectType)(rand() % Simulation::PhysicsObjectType::OBJ_NUM_TOTAL), m_objectIDCounter++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRangeVec(0.5f, 1.f));
		}
	}
	else
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			SpawnObjectAt(typeOfObjects, m_objectIDCounter++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRangeVec(0.5f, 1.f));
		}
	}
}

void Managers::BachelorSimulationManager::SpawnObjectsFromScenario(const Simulation::Scenario *scenario)
{
	for (auto desc : scenario->GetObjectDescriptions())
	{
		SceneObject *obj = SpawnObjectAt(desc.m_objectType, desc.m_ID, desc.m_initialPosition, desc.m_initialRotation, desc.m_initialRotationAngle, desc.m_initialScale);
		obj->SetTranslationStep(desc.m_translationStep);
		obj->SetScaleStep(desc.m_scaleStep);
		obj->SetRotationStep(desc.m_rotationStep);
		obj->SetRotationAngleStep(desc.m_rotationAngleStep);
		m_firstAvailableID = desc.m_ID > m_firstAvailableID ? desc.m_ID : m_firstAvailableID;
	}

	m_firstAvailableID++;
}

Rendering::SceneObject* Managers::BachelorSimulationManager::SpawnObjectAt(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale)
{
	Rendering::Models::Model *newObj;

	switch (objectType)
	{
	case Simulation::PhysicsObjectType::OBJ_CUBE:
		newObj = new Rendering::Models::Cube(m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_TETRAHEDRON:
		newObj = new Rendering::Models::Tetrahedron(m_modelManager, this);
		break;

	case Simulation::PhysicsObjectType::OBJ_SPHERE:
		newObj = new Rendering::Models::Sphere(m_modelManager, this);
		break;
	case Simulation::PhysicsObjectType::OBJ_CYLINDER:
		newObj = new Rendering::Models::Cylinder(m_modelManager, this);
		break;
	default:
		newObj = new Rendering::Models::Cone(m_modelManager, this);
		break;
	}

	newObj->SetID(ID);
	newObj->Create();
	newObj->ScaleAbsolute(scale);
	newObj->RotateAbsolute(rotation, rotationAngle);
	newObj->TranslateAbsolute(position);
	ObjectAdded(newObj);
	ObjectMoved(newObj);
	newObj->ObjectMoved();

	m_modelManager->RegisterObject(ID, newObj);

	return newObj;
}
