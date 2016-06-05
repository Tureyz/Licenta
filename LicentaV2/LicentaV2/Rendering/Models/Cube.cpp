#include "Cube.h"

Rendering::Models::Cube::Cube(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager) : Model(color, modelManager, simulationManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
	SetObjectType(0);
}

Rendering::Models::Cube::~Cube()
{
}

void Rendering::Models::Cube::Create()
{
	this->m_vao = GetModelManager()->m_cubeVao;
	this->m_vbos.push_back(GetModelManager()->m_cubeVbo);
	this->m_vbos.push_back(GetModelManager()->m_cubeIbo);
	this->SetIndices(GetModelManager()->m_cubeIndices);
	this->m_initialVertices = GetModelManager()->m_cubeVerts;
	this->m_transformedVertices = GetModelManager()->m_cubeVerts;
	UpdateVertices(glm::mat4(1.0f));
	GetBoundingBox()->Create();
}