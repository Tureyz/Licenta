#include "Cylinder.h"

Rendering::Models::Cylinder::Cylinder(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager) : Model(color, modelManager, simulationManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
	SetObjectType(Simulation::PhysicsObjectType::OBJ_CYLINDER);
}

Rendering::Models::Cylinder::~Cylinder()
{

}

void Rendering::Models::Cylinder::Create()
{
	this->m_vao = GetModelManager()->m_cylinderVao;
	this->m_vbos.push_back(GetModelManager()->m_cylinderVbo);
	this->m_vbos.push_back(GetModelManager()->m_cylinderIbo);
	this->SetIndices(GetModelManager()->m_cylinderIndices);
	this->m_initialVertices = GetModelManager()->m_cylinderVerts;
	this->m_transformedVertices = GetModelManager()->m_cylinderVerts;
	UpdateVertices(glm::mat4(1.0f));
	GetBoundingBox()->Create();
}
