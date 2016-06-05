#include "Cone.h"

Rendering::Models::Cone::Cone(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager) : Model(color, modelManager, simulationManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
	SetObjectType(Simulation::PhysicsObjectType::OBJ_CONE);
}

Rendering::Models::Cone::~Cone()
{

}

void Rendering::Models::Cone::Create()
{
	this->m_vao = GetModelManager()->m_coneVao;
	this->m_vbos.push_back(GetModelManager()->m_coneVbo);
	this->m_vbos.push_back(GetModelManager()->m_coneIbo);
	this->SetIndices(GetModelManager()->m_coneIndices);
	this->m_initialVertices = GetModelManager()->m_coneVerts;
	this->m_transformedVertices = GetModelManager()->m_coneVerts;
	UpdateVertices(glm::mat4(1.0f));
	GetBoundingBox()->Create();
}
