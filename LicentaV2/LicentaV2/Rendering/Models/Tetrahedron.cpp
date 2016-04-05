#include "Tetrahedron.h"

Rendering::Models::Tetrahedron::Tetrahedron(const glm::vec4 & color, Managers::ModelManager *modelManager) : Model(color, modelManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
}

Rendering::Models::Tetrahedron::~Tetrahedron()
{
}

void Rendering::Models::Tetrahedron::Create()
{
	this->m_vao = GetModelManager()->m_tetraVao;
	this->m_vbos.push_back(GetModelManager()->m_tetraVbo);
	this->m_vbos.push_back(GetModelManager()->m_tetraIbo);
	this->m_verticesSize = GetModelManager()->m_tetraVerts.size();
	this->m_initialVertices = GetModelManager()->m_tetraVerts;
	this->m_transformedVertices = GetModelManager()->m_tetraVerts;
	UpdateVertices(glm::mat4(1.0f));
	//GetBoundingBox()->SetProgram(m_program);
	GetBoundingBox()->Create();
}

void Rendering::Models::Tetrahedron::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{

	//For now, all objects use the same shaders
	//glUseProgram(m_program);

	glUniform1i(2, GetCollisionState());
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * m_modelMatrix;
	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);
	glBindVertexArray(m_vao);

	glDrawElements(GL_TRIANGLE_STRIP, 6, GL_UNSIGNED_INT, 0);

	DrawBB(projectionMatrix, viewMatrix);
}

void Rendering::Models::Tetrahedron::Update()
{
	//m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
}

void Rendering::Models::Tetrahedron::DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix)
{
	GetBoundingBox()->Draw(projection_matrix, view_matrix);
}
