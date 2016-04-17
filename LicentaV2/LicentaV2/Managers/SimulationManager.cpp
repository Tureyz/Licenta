#include "SimulationManager.h"
#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Sphere.h"
#include "../Rendering/Models/Tetrahedron.h"

Managers::SimulationManager::SimulationManager(ModelManager *modelManager)
{
	m_objectIDCounter = 0;
	m_time = m_timeBase = 0;
	m_modelManager = modelManager;
	m_allObjects = m_modelManager->GetModelListPtr();
	m_collisionDebug = false;
}

Managers::SimulationManager::~SimulationManager()
{

}

void Managers::SimulationManager::Init()
{
	SpawnManyAround(glm::vec3(0.f, 0.f, 0.f), 7.f, 1500, Managers::physicsObjectType::OBJ_RANDOM);
	m_modelManager->SetBoundingBoxesVisibile(false);
	
	m_collisionMethods["BVH"] = new Collision::BVH(m_allObjects);
	m_collisionMethods["BVH"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	
	m_collisionMethods["Octree"] = new Collision::Octree(m_allObjects, glm::vec3(-10, -10, -10), glm::vec3(10, 10, 10));
	m_collisionMethods["Octree"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::Octree *)m_collisionMethods["Octree"])->SetParams(5, 50);

	m_collisionMethods["Spatial Grid"] = new Collision::SpatialGrid(m_allObjects);
	m_collisionMethods["Spatial Grid"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::SpatialGrid *)m_collisionMethods["Spatial Grid"])->SetParams(glm::vec3(-10, -10, -10), glm::vec3(10, 10, 10), 5);

	m_collisionMethods["Sweep and Prune"] = new Collision::SweepAndPrune(m_allObjects);
	m_collisionMethods["Sweep and Prune"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);

	m_collisionMethods["Spatial Hashing"] = new Collision::SpatialHashing(m_allObjects);
	m_collisionMethods["Spatial Hashing"]->SetDrawBuffers(m_modelManager->m_cubeVao, m_modelManager->m_cubeVbo, m_modelManager->m_cubeIbo);
	((Collision::SpatialHashing *)m_collisionMethods["Spatial Hashing"])->SetCellSize(5);

	//m_activeMethod = m_collisionMethods["Sweep and Prune"];
	m_activeMethod = m_collisionMethods.find("Spatial Grid");
	//(*m_activeMethod).second->SetShowDebug(true);

	std::cout << "Current method: " << (*m_activeMethod).first.c_str() << std::endl;
}

void Managers::SimulationManager::Update()
{
	(*m_activeMethod).second->SetShowDebug(m_collisionDebug);

	// Collision checks 30 times per second
	m_time = glutGet(GLUT_ELAPSED_TIME);
	if (m_time - m_timeBase > 1000.f / 30.f)
	{
		m_timeBase = m_time;
		//std::cout << "TICK\n";
		(*m_activeMethod).second->Update();
		TestCollision();
	}
}

void Managers::SimulationManager::Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	(*m_activeMethod).second->DrawDebug(projectionMatrix, viewMatrix);
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
		std::cout << "Collision Debug " << (m_collisionDebug ? "ON" : "OFF") << std::endl;
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

	std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = (*m_activeMethod).second->TestCollision();
	
	for (int i = 0; i < asd.size(); ++i)
	{
		asd[i].first->SetCollisionState(COLLIDING);
		asd[i].second->SetCollisionState(COLLIDING);
	}
}

void Managers::SimulationManager::ResetCollisions()
{
	for (auto obj : *m_allObjects)
	{
		obj->SetCollisionState(DEFAULT);
	}
}

glm::vec3 Managers::SimulationManager::RandomPositionAround(const glm::vec3 &position, const float radius)
{

	return glm::vec3
	(
		position.x - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius))),
		position.y - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius))),
		position.z - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius)))
	);
}

glm::vec3 Managers::SimulationManager::RandomRotationAxis()
{
	return glm::vec3
	(
		(std::rand() % 1000) / 1000.f,
		(std::rand() % 1000) / 1000.f,
		(std::rand() % 1000) / 1000.f
	);
}

glm::vec3 Managers::SimulationManager::RandomScale(float min, float max)
{
	return glm::vec3
	(
		min + (float)(std::rand()) / (float(RAND_MAX / (max - min))),
		min + (float)(std::rand()) / (float(RAND_MAX / (max - min))),
		min + (float)(std::rand()) / (float(RAND_MAX / (max - min)))
	);
}

void Managers::SimulationManager::SpawnManyAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Managers::physicsObjectType typeOfObjects)
{
	if (typeOfObjects == OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = RandomPositionAround(position, radius);
			SpawnObjectAt(pos, (physicsObjectType)(rand() % 3), glm::vec4(0.7, 0.7, 0.7, 1));
		}
	}
	else
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = RandomPositionAround(position, radius);
			SpawnObjectAt(pos, typeOfObjects, glm::vec4(0.7, 0.7, 0.7, 1));
		}
	}
}

void Managers::SimulationManager::SpawnObjectAt(const glm::vec3 &position, const physicsObjectType objectType, const glm::vec4 &color)
{
	Rendering::Models::Model *newObj;

	switch (objectType)
	{
	case OBJ_CUBE:
		newObj = new Rendering::Models::Cube(color, m_modelManager);
		break;
	case OBJ_TETRAHEDRON:
		newObj = new Rendering::Models::Tetrahedron(color, m_modelManager);
		break;
	default:
		newObj = new Rendering::Models::Sphere(color, m_modelManager);
		break;
	}

	newObj->SetID(m_objectIDCounter);
	newObj->Create();
	float angle = (float)(std::rand() % 360);
	newObj->ScaleAbsolute(RandomScale(0.5f, 1.f));
	newObj->RotateAbsolute(RandomRotationAxis(), angle);
	newObj->TranslateAbsolute(position);

	m_modelManager->SetModel(m_objectIDCounter++, newObj);
}
