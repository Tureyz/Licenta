#include "Sphere.h"

Rendering::Models::Sphere::Sphere(const glm::vec4 & color, Managers::ModelManager *modelManager) : Model(color, modelManager)
{
	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
}

Rendering::Models::Sphere::~Sphere()
{

}

void Rendering::Models::Sphere::Create()
{
	this->m_vao = GetModelManager()->m_sphereVao;
	this->m_vbos.push_back(GetModelManager()->m_sphereVbo);
	this->m_vbos.push_back(GetModelManager()->m_sphereIbo);
	this->m_indices = GetModelManager()->m_sphereIndices;
	this->m_initialVertices = GetModelManager()->m_sphereVerts;
	this->m_transformedVertices = GetModelManager()->m_sphereVerts;
	UpdateVertices(glm::mat4(1.0f));
	//GetBoundingBox()->SetProgram(m_program);
	GetBoundingBox()->Create();
}

void Rendering::Models::Sphere::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	//For now, all objects use the same shaders
	//glUseProgram(m_program);

	glUniform1i(2, GetCollisionState());
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * m_modelMatrix;
	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);
	glBindVertexArray(m_vao);

	//glDrawArrays(GL_TRIANGLES, 0, 36);
	glDrawElements(GL_TRIANGLES, (GLsizei)m_indices.size(), GL_UNSIGNED_INT, &m_indices[0]);

	DrawBB(projectionMatrix, viewMatrix);
}

void Rendering::Models::Sphere::Update()
{
}

void Rendering::Models::Sphere::DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix)
{
	GetBoundingBox()->Draw(projection_matrix, view_matrix);
}
