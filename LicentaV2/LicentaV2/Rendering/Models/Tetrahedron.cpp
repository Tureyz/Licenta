#include "Tetrahedron.h"

Rendering::Models::Tetrahedron::Tetrahedron(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager) : Model(color, modelManager, simulationManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
	SetObjectType(2);
}

Rendering::Models::Tetrahedron::~Tetrahedron()
{
}

void Rendering::Models::Tetrahedron::Create()
{
	this->m_vao = GetModelManager()->m_tetraVao;
	this->m_vbos.push_back(GetModelManager()->m_tetraVbo);
	this->m_vbos.push_back(GetModelManager()->m_tetraIbo);
	this->SetIndices(GetModelManager()->m_tetraIndices);
	this->m_verticesSize = GetModelManager()->m_tetraVerts.size();
	this->m_initialVertices = GetModelManager()->m_tetraVerts;
	this->m_transformedVertices = GetModelManager()->m_tetraVerts;
	UpdateVertices(glm::mat4(1.0f));
	//GetBoundingBox()->SetProgram(m_program);
	GetBoundingBox()->Create();
}
