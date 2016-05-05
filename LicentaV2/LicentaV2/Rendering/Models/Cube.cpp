#include "Cube.h"

using namespace Rendering;
using namespace Models;

#define PI 3.14159265

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
	this->m_initialVertices = GetModelManager()->m_cubeVerts;
	this->m_transformedVertices = GetModelManager()->m_cubeVerts;
	UpdateVertices(glm::mat4(1.0f));
	//GetBoundingBox()->SetProgram(m_program);
	GetBoundingBox()->Create();
}

void Rendering::Models::Cube::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{

	//For now, all objects use the same shaders
	//glUseProgram(m_program);

	glUniform1i(2, GetCollisionState());
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * m_modelMatrix;
	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);

	glBindVertexArray(m_vao);
	//glDrawArrays(GL_TRIANGLES, 0, 36);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

	DrawBB(projectionMatrix, viewMatrix);
}

void Rendering::Models::Cube::Update()
{
	Rendering::Models::Model::Update();
}

void Rendering::Models::Cube::DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix)
{
	GetBoundingBox()->Draw(projection_matrix, view_matrix);
}
