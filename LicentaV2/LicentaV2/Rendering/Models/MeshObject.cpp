#include "MeshObject.h"

Rendering::Models::MeshObject::MeshObject(int rows, int cols, Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager), m_rows(rows), m_cols(cols)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_MESH);	
}


Rendering::Models::MeshObject::~MeshObject()
{
	delete m_clothBehavior;
}

void Rendering::Models::MeshObject::Create()
{
	Create(glm::mat4(1.0f));
}

void Rendering::Models::MeshObject::Create(const glm::mat4 &mvp)
{
	m_visualBody = m_modelManager->CreateMeshVisualBody(m_rows, m_cols);
	m_boundingBox.CreateVisualBody(m_modelManager->CreateBasicVisualBody(Simulation::PhysicsObjectType::OBJ_CUBE));

	UpdateVertices(mvp);
	SetCollisionData(new Collision::DataStructures::CollisionData(&m_visualBody.m_verts, &m_visualBody.m_indices));
	m_clothBehavior = new Collision::DataStructures::ClothBehavior(m_collisionData);
	//m_collisionData->m_narrowPhaseMethod->SetModelManager(m_modelManager);
	
	//m_collisionData->CreateTriangles();
}
