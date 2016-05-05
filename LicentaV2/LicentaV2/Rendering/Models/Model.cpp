#include "Model.h"
#include <algorithm>
#include "..\..\Core\Utils.hpp"

Rendering::Models::Model::Model(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager)
{
	m_translationMatrix = glm::mat4(1.0f);
	m_rotationMatrix = glm::mat4(1.0f);
	m_scaleMatrix = glm::mat4(1.0f);
	SetModelManager(modelManager);
	SetSimulationManager(simulationManager);
	SetScale(glm::vec3(1.f));
	SetPosition(glm::vec3(0.f));
	SetRotation(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetCollisionState(DEFAULT);
	SetColor(color);

	SetRotationStep(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetScaleStep(glm::vec3(0.f));
	SetTranslationStep(glm::vec3(0.f));
}

Rendering::Models::Model::~Model()
{
	Destroy();

	if (m_auxCollisionData)
	{
		delete m_auxCollisionData;
		m_auxCollisionData = NULL;
	}
}

void Rendering::Models::Model::Create()
{

}

void Rendering::Models::Model::Draw()
{
}

void Rendering::Models::Model::Draw(const glm::mat4 & projection_matrix, const glm::mat4 & view_matrix)
{
}

void Rendering::Models::Model::Update()
{
	if (m_matrixChanged)
	{
		ObjectMoved();
		m_matrixChanged = false;
	}

	if (GetScaleStep() != glm::vec3(1.f))
		ScaleRelative(GetScaleStep() * Core::Utils::DeltaTime());
	if (m_rotationAngleStep != 0.f)
		RotateRelative(GetRotationStep(), m_rotationAngleStep * Core::Utils::DeltaTime());
	if (GetTranslationStep() != glm::vec3(0.f))
		TranslateRelative(GetTranslationStep() * Core::Utils::DeltaTime());
}

void Rendering::Models::Model::SetProgram(GLuint shaderName)
{
	this->m_program = shaderName;
}

void Rendering::Models::Model::Destroy()
{
	//glDeleteVertexArrays(1, &m_vao);
	//glDeleteBuffers((GLsizei) m_vbos.size(), &m_vbos[0]);
	//m_vbos.clear();

	delete m_boundingBox;
}

GLuint Rendering::Models::Model::GetVao() const
{
	return m_vao;
}

const std::vector<GLuint>& Rendering::Models::Model::GetVbos() const
{
	// TODO: insert return statement here
	return m_vbos;
}

void Rendering::Models::Model::TranslateAbsolute(const glm::vec3 & pos)
{
	m_translationMatrix = glm::translate(glm::mat4(1), pos);
	SetPosition(pos);	
	m_matrixChanged = true;
}

void Rendering::Models::Model::RotateAbsolute(const glm::vec3 &axis, const float angles)
{
	m_rotationMatrix = glm::rotate(glm::mat4(1), angles, axis);
	SetRotation(axis);
	SetRotationAngle(angles);
	m_matrixChanged = true;
}

void Rendering::Models::Model::ScaleAbsolute(const glm::vec3 &scales)
{
	m_scaleMatrix = glm::scale(glm::mat4(1), scales);
	SetScale(scales);
	m_matrixChanged = true;
}

void Rendering::Models::Model::TranslateRelative(const glm::vec3 & pos)
{
	m_translationMatrix = glm::translate(m_translationMatrix, pos);
	SetPosition(GetPosition() + pos);
	m_matrixChanged = true;
}

void Rendering::Models::Model::RotateRelative(const glm::vec3 &axis, const float angles)
{
	m_rotationMatrix = glm::rotate(m_rotationMatrix, angles, axis);
	SetRotation(GetRotation() + axis);
	SetRotationAngle(GetRotationAngle() + angles);
	m_matrixChanged = true;
}

void Rendering::Models::Model::ScaleRelative(const glm::vec3 &scales)
{
	m_scaleMatrix = glm::scale(m_scaleMatrix, scales);
	SetScale(GetScale() + scales);
	m_matrixChanged = true;
}

void Rendering::Models::Model::DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix)
{
}

void Rendering::Models::Model::UpdateVertices(glm::mat4 mat)
{	
	glm::vec4 asd = mat * glm::vec4(m_initialVertices[0].m_position, 1);
	m_transformedVertices[0].m_position = glm::vec3(asd.x, asd.y, asd.z);
	m_minCoords = m_maxCoords = m_transformedVertices[0].m_position;

	for (int i = 1; i < m_initialVertices.size(); ++i)
	{
		asd = mat * glm::vec4(m_initialVertices[i].m_position, 1);
		//std::cout << "BEFORE: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;
		
		m_transformedVertices[i].m_position = glm::vec3(asd.x, asd.y, asd.z);
		if (asd.x < m_minCoords.x)
			m_minCoords.x = asd.x;
		if (asd.y < m_minCoords.y)
			m_minCoords.y = asd.y;
		if (asd.z < m_minCoords.z)
			m_minCoords.z = asd.z;

		if (asd.x > m_maxCoords.x)
			m_maxCoords.x = asd.x;
		if (asd.y > m_maxCoords.y)
			m_maxCoords.y = asd.y;
		if (asd.z > m_maxCoords.z)
			m_maxCoords.z = asd.z;
		//std::cout << "AFTER: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;
	}
}

void Rendering::Models::Model::SetBoundingBoxVisible(bool value)
{
	this->GetBoundingBox()->SetVisible(value);
}

void Rendering::Models::Model::ObjectMoved()
{
	m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
	UpdateVertices(m_modelMatrix);
	GetBoundingBox()->UpdateValues();
	m_simulationManager->ObjectMoved(this);
}

