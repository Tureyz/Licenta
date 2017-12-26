#include "MastersSimulationManager.h"
#include "../Core/Utils.hpp"
#include "../Collision/BVH.h"

#include "../Rendering/VisualBodyFactory.h"
#include "../Physics/DeformableBody.h"
#include "../Physics/RigidBody.h"

Managers::MastersSimulationManager::MastersSimulationManager(Managers::ModelManager *modelManager)
{
	m_objectIDCounter = 0;
	m_modelManager = modelManager;
	m_allObjects = m_modelManager->GetModelListPtr();
	m_objectBBsVisible = true;
	m_broadPhaseDebugDraw = false;
	m_narrowPhaseDebugDraw = false;

}

void Managers::MastersSimulationManager::Init()
{
	m_broadPhaseMethod = new Collision::BVH(m_allObjects);
	m_broadPhaseMethod->SetShowDebug(m_broadPhaseDebugDraw);

	Rendering::SceneObject *meshObj = new Rendering::SceneObject();
	meshObj->SetID(m_objectIDCounter);
	meshObj->SetBoundingBox(new Collision::DataStructures::BoundingBox());
	meshObj->GetBoundingBox()->CreateVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateBasicVisualBody(Rendering::VisualBodyType::OBJ_LINE_CUBE));
	meshObj->SetVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateMeshVisualBody(20, 20));	
	meshObj->UpdateVertices(glm::mat4(1.0f));
	meshObj->SetPhysicsBody(new Physics::DeformableBody(&meshObj->GetVisualBody()->m_verts, &meshObj->GetVisualBody()->m_indices));

	ObjectAdded(meshObj);
	ObjectMoved(meshObj);
	meshObj->ObjectMoved();
	m_modelManager->RegisterObject(m_objectIDCounter++, meshObj);


	Rendering::SceneObject *sphereObj = new Rendering::SceneObject();
	sphereObj->SetID(m_objectIDCounter);
	sphereObj->SetBoundingBox(new Collision::DataStructures::BoundingBox());
	sphereObj->GetBoundingBox()->CreateVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateBasicVisualBody(Rendering::VisualBodyType::OBJ_LINE_CUBE));
	sphereObj->SetVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateBasicVisualBody(Rendering::VisualBodyType::OBJ_SPHERE));
	sphereObj->UpdateVertices(glm::mat4(1.0f));
	sphereObj->SetPhysicsBody(new Physics::RigidBody(&sphereObj->GetVisualBody()->m_verts, &sphereObj->GetVisualBody()->m_indices));

	sphereObj->TranslateAbsolute(glm::vec3(-0.25f, -0.25f, 0));
 	sphereObj->SetTranslationStep(glm::vec3(0.00005f, 0.00005f, 0.f));

	ObjectAdded(sphereObj);
	ObjectMoved(sphereObj);
	sphereObj->ObjectMoved();
	m_modelManager->RegisterObject(m_objectIDCounter++, sphereObj);



	m_modelManager->SetBoundingBoxesVisibile(m_objectBBsVisible);
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
			Collision::NarrowBVH *method = new Collision::NarrowBVH(candidatePair.first->GetPhysicsBody()->GetTrianglesPtr());
			method->Update();
			m_narrowMethods[candidatePair.first->GetID()] = method;
		}

		if (!m_narrowMethods.count(candidatePair.second->GetID()))
		{
			Collision::NarrowBVH *method = new Collision::NarrowBVH(candidatePair.second->GetPhysicsBody()->GetTrianglesPtr());
			method->Update();
			m_narrowMethods[candidatePair.second->GetID()] = method;
		}


		std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>> collidingTriangles = m_narrowMethods[candidatePair.first->GetID()]->TestCollision(m_narrowMethods[candidatePair.second->GetID()]);


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

void Managers::MastersSimulationManager::ObjectMoved(Rendering::SceneObject *object)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->ObjectMoved(object);
}

void Managers::MastersSimulationManager::ObjectAdded(Rendering::SceneObject *object)
{
	if (m_broadPhaseMethod)
		m_broadPhaseMethod->ObjectAdded(object);
}

void Managers::MastersSimulationManager::ObjectRemoved(Rendering::SceneObject *object)
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

void Managers::MastersSimulationManager::BreakObject(Rendering::SceneObject *obj, glm::vec3 impactForce)
{
}

std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> Managers::MastersSimulationManager::GetBroadPhasePairs()
{
	return m_broadPhaseMethod->TestCollision();
}

void Managers::MastersSimulationManager::Draw()
{
}

