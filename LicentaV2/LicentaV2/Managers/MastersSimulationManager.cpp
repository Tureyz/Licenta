#include "MastersSimulationManager.h"
#include "..\Rendering\Models\Sphere.h"

#include "..\Rendering\Models\MeshObject.h"
#include "../Core/Utils.hpp"
#include "..\Collision\BVH.h"

Managers::MastersSimulationManager::MastersSimulationManager(Managers::ModelManager *modelManager)
{
	m_objectIDCounter = 0;
	m_modelManager = modelManager;
	m_allObjects = m_modelManager->GetModelListPtr();
	m_objectBBsVisible = true;
	m_broadPhaseDebugDraw = true;
	m_narrowPhaseDebugDraw = true;

}

void Managers::MastersSimulationManager::Init()
{
	m_broadPhaseMethod = new Collision::BVH(m_allObjects);
	m_broadPhaseMethod->SetShowDebug(m_broadPhaseDebugDraw);
	m_broadPhaseMethod->SetModelManager(m_modelManager);

	Rendering::Models::MeshObject *testObj = new Rendering::Models::MeshObject(100, 100, m_modelManager, this);
	testObj->SetID(m_objectIDCounter);
	testObj->Create();
	//testObj->GetCollisionData()->m_narrowPhaseMethod->SetShowDebug(m_narrowPhaseDebugDraw);

	//testObj->ScaleAbsolute(glm::vec3(1.f));
	//testObj->RotateAbsolute(glm::vec3(0.f, 1.f, 0.f), 15);
	//testObj->TranslateAbsolute(glm::vec3(0.1f));

	//testObj->SetScaleStep(glm::vec3(1.f));
	//testObj->SetRotationAngleStep(0.f);
	//testObj->SetTranslationStep(glm::vec3(0.f));

	ObjectAdded(testObj);
	ObjectMoved(testObj);
	testObj->ObjectMoved();
	m_modelManager->RegisterObject(m_objectIDCounter++, testObj);



	Rendering::Models::Sphere *sphereObj = new Rendering::Models::Sphere(m_modelManager, this);
	sphereObj->SetID(m_objectIDCounter);
	sphereObj->Create();
	sphereObj->TranslateAbsolute(glm::vec3(-0.25f, -0.25f, 0));

// 	sphereObj->ScaleAbsolute(glm::vec3(0.25f));
// 	sphereObj->RotateAbsolute(glm::vec3(1.f), 0);
// 	sphereObj->TranslateAbsolute(glm::vec3(-0.25f, -0.25f, 0.05f));
// 
// 	sphereObj->SetScaleStep(glm::vec3(1.f));
// 	sphereObj->SetRotationStep(glm::vec3(0.f, 1.f, 0.f));
// 	sphereObj->SetRotationAngleStep(0.01f);
// 	sphereObj->SetTranslationStep(glm::vec3(0.00005f, 0.00005f, 0.f));

	ObjectAdded(sphereObj);
	ObjectMoved(sphereObj);
	sphereObj->ObjectMoved();
	m_modelManager->RegisterObject(m_objectIDCounter++, sphereObj);

// 	Rendering::Models::Sphere *sphereObj2 = new Rendering::Models::Sphere(m_modelManager, this);
// 	sphereObj2->SetID(m_objectIDCounter);
// 	sphereObj2->Create();
// 
// 	sphereObj2->ScaleAbsolute(glm::vec3(0.1f));
// 	sphereObj2->RotateAbsolute(glm::vec3(1.f), 0);
// 	sphereObj2->TranslateAbsolute(glm::vec3(-1.f, -1.f, 0.05f));
// 
// 	sphereObj2->SetScaleStep(glm::vec3(1.f));
// 	sphereObj2->SetRotationAngleStep(0.f);
// 	sphereObj2->SetTranslationStep(glm::vec3(0.001f, 0.001f, 0.f));
// 
// 	ObjectAdded(sphereObj2);
// 	ObjectMoved(sphereObj2);
// 	sphereObj2->ObjectMoved();
// 	m_modelManager->RegisterObject(m_objectIDCounter++, sphereObj2);



	m_modelManager->SetBoundingBoxesVisibile(false);

}

void Managers::MastersSimulationManager::FixedUpdate()
{
	m_broadPhaseMethod->Update();


	for (auto kvPair : m_narrowMethods)
	{
		kvPair.second->Update();
	}

	auto broadPhasePairs = GetBroadPhasePairs();

	for (auto candidatePair : broadPhasePairs)
	{
		//std::cout << candidatePair.first->GetID() << " + " << candidatePair.second->GetID() << std::endl;

		if (!m_narrowMethods.count(candidatePair.first->GetID()))
		{
			Collision::NarrowBVH *method = new Collision::NarrowBVH(&(candidatePair.first->GetCollisionData()->m_triangles));
			method->Update();
			m_narrowMethods[candidatePair.first->GetID()] = method;
		}

		if (!m_narrowMethods.count(candidatePair.second->GetID()))
		{
			Collision::NarrowBVH *method = new Collision::NarrowBVH(&(candidatePair.second->GetCollisionData()->m_triangles));
			method->Update();
			m_narrowMethods[candidatePair.second->GetID()] = method;
		}


		std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> collidingTriangles = m_narrowMethods[candidatePair.first->GetID()]->TestCollision(m_narrowMethods[candidatePair.second->GetID()]);


		for (auto trianglePair : collidingTriangles)
		{
			trianglePair.first->SetColliding();
			trianglePair.second->SetColliding();
		}

	}

}

void Managers::MastersSimulationManager::Update()
{
}

void Managers::MastersSimulationManager::Draw(const glm::mat4 &viewProjection)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->DrawDebug(viewProjection);
}

void Managers::MastersSimulationManager::ObjectMoved(Rendering::IPhysicsObject *object)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->ObjectMoved(object);
}

void Managers::MastersSimulationManager::ObjectAdded(Rendering::IPhysicsObject *object)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->ObjectAdded(object);
}

void Managers::MastersSimulationManager::ObjectRemoved(Rendering::IPhysicsObject *object)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->ObjectRemoved(object);
}

void Managers::MastersSimulationManager::KeyPressed(unsigned char key)
{
}

void Managers::MastersSimulationManager::KeyReleased(unsigned char key)
{
}

void Managers::MastersSimulationManager::MousePressed(int button, int state, int x, int y)
{
}

void Managers::MastersSimulationManager::MouseMove(int x, int y, int width, int height)
{
}

void Managers::MastersSimulationManager::BreakObject(Rendering::IPhysicsObject *obj, glm::vec3 impactForce)
{
}

std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Managers::MastersSimulationManager::GetBroadPhasePairs()
{
	return m_broadPhaseMethod->TestCollision();
}

void Managers::MastersSimulationManager::Draw()
{
}

