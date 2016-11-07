#include "Sphere.h"

Rendering::Models::Sphere::Sphere(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager) : Model(color, modelManager, simulationManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
	SetObjectType(1);	
}

Rendering::Models::Sphere::~Sphere()
{

}

void Rendering::Models::Sphere::Create()
{
	this->m_vao = GetModelManager()->m_sphereVao;
	this->m_vbos.push_back(GetModelManager()->m_sphereVbo);
	this->m_vbos.push_back(GetModelManager()->m_sphereIbo);
	this->SetIndices(GetModelManager()->m_sphereIndices);
	this->m_initialVertices = GetModelManager()->m_sphereVerts;
	this->m_transformedVertices = GetModelManager()->m_sphereVerts;	
	UpdateVertices(glm::mat4(1.0f));
	//GetBoundingBox()->SetProgram(m_program);
	GetBoundingBox()->Create();
}