#include "BoundingBox.h"
#include "../../Managers/ModelManager.h"
#include "../../Rendering/Models/Model.h"

Collision::DataStructures::BoundingBox::BoundingBox(Rendering::IPhysicsObject * parentObject)
{
	m_parentObject = parentObject;
	m_parentObjects = NULL;
	m_isVisible = false;
	m_color = glm::vec4(0.2, 0.3, 0.4, 1);
	m_numObjects = 1;
}

Collision::DataStructures::BoundingBox::BoundingBox(Rendering::IPhysicsObject ** parentObjects, size_t numObjects)
{
	m_isVisible = false;
	m_color = glm::vec4(0.2, 0.3, 0.4, 1);
	m_parentObjects = parentObjects;
	m_numObjects = numObjects;
	m_parentObject = NULL;
}

Collision::DataStructures::BoundingBox::BoundingBox()
{
	m_parentObjects = NULL;
	m_parentObject = NULL;
	m_isVisible = false;
}

Collision::DataStructures::BoundingBox::~BoundingBox()
{
	Destroy();
}

void Collision::DataStructures::BoundingBox::Create()
{
	Rendering::Models::Model *parent;
	if (m_parentObject)
	{
		parent = (Rendering::Models::Model *)m_parentObject;
	}
	else
	{
		parent = (Rendering::Models::Model *)m_parentObjects[0];
	}

	if (!parent)
	{
		std::cout << "EROARE BA\n";
	}

	this->m_vao = parent->GetModelManager()->m_cubeVao;
	this->m_vbos.push_back(parent->GetModelManager()->m_cubeVbo);
	this->m_vbos.push_back(parent->GetModelManager()->m_cubeIbo);
	UpdateValues();
}


void Collision::DataStructures::BoundingBox::Update()
{
// 	std::cout << "PARENT POS: " << m_parentObject->GetPosition().x << " " << m_parentObject->GetPosition().y << " " << m_parentObject->GetPosition().z << std::endl;
// 	TranslateAbsolute(m_parentObject->GetPosition());
// 	ScaleAbsolute(m_parentObject->GetScale());
// 	m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
}

void Collision::DataStructures::BoundingBox::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	if (!m_isVisible)
		return;

	glm::vec3 size = glm::vec3(m_maxX - m_minX, m_maxY - m_minY, m_maxZ - m_minZ);
	glm::vec3 center = glm::vec3((m_maxX + m_minX) / 2, (m_maxY + m_minY) / 2, (m_maxZ + m_minZ) / 2);
	glm::mat4 modelMatrix = glm::translate(glm::mat4(1), center) * glm::scale(glm::mat4(1), size);

	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * modelMatrix;

	//For now, all objects use the same shaders
	//glUseProgram(m_program);

	glUniform1i(2, Rendering::BOUNDINGBOX);
	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);
	glBindVertexArray(m_vao);

	//glDrawArrays(GL_TRIANGLES, 0, 36);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

void Collision::DataStructures::BoundingBox::Destroy()
{
// 	glDeleteVertexArrays(1, &m_vao);
// 	glDeleteBuffers((GLsizei)m_vbos.size(), &m_vbos[0]);
// 	m_vbos.clear();
}

void Collision::DataStructures::BoundingBox::UpdateValues()
{
	if (m_parentObject)
	{
		m_minX = m_parentObject->GetMinCoords().x;
		m_maxX = m_parentObject->GetMaxCoords().x;

		m_minY = m_parentObject->GetMinCoords().y;
		m_maxY = m_parentObject->GetMaxCoords().y;

		m_minZ = m_parentObject->GetMinCoords().z;
		m_maxZ = m_parentObject->GetMaxCoords().z;
	}
	else
	{
		m_minX = m_parentObjects[0]->GetMinCoords().x;
		m_maxX = m_parentObjects[0]->GetMaxCoords().x;

		m_minY = m_parentObjects[0]->GetMinCoords().y;
		m_maxY = m_parentObjects[0]->GetMaxCoords().y;

		m_minZ = m_parentObjects[0]->GetMinCoords().z;
		m_maxZ = m_parentObjects[0]->GetMaxCoords().z;

		for (int i = 0; i < m_numObjects; ++i)
		{
			glm::vec3 minCoords = m_parentObjects[i]->GetMinCoords();
			glm::vec3 maxCoords = m_parentObjects[i]->GetMaxCoords();

			if (minCoords.x < m_minX) m_minX = minCoords.x;
			if (minCoords.y < m_minY) m_minY = minCoords.y;
			if (minCoords.z < m_minZ) m_minZ = minCoords.z;

			if (maxCoords.x > m_maxX) m_maxX = maxCoords.x;
			if (maxCoords.y > m_maxY) m_maxY = maxCoords.y;
			if (maxCoords.z > m_maxZ) m_maxZ = maxCoords.z;
		}
	}		
}

void Collision::DataStructures::BoundingBox::SetProgram(GLuint shaderName)
{
	this->m_program = shaderName;
}

bool Collision::DataStructures::BoundingBox::Collides(const BoundingBox * other)
{
// 	if () return false;
// 	if () return false;
// 
// 	if (m_maxY < other->m_minY) return false;
// 	if (m_minY > other->m_maxY) return false;
// 
// 	if (m_maxZ < other->m_minZ) return false;
// 	if (m_minZ > other->m_maxZ) return false;
	
	return !(m_maxX < other->m_minX || m_minX > other->m_maxX || m_maxY < other->m_minY || m_minY > other->m_maxY || m_maxZ < other->m_minZ || m_minZ > other->m_maxZ);
	//return true;
}

