#include "BoundingBox.h"
#include "../../Core/Utils.hpp"

Collision::DataStructures::BoundingBox::BoundingBox()
{
	SetVisible(false);
}

Collision::DataStructures::BoundingBox::~BoundingBox()
{	
}

void Collision::DataStructures::BoundingBox::CreateVisualBody(Rendering::VisualBody &visualBody)
{
	m_visualBody = visualBody;


	for (int i = 0; i < m_visualBody.m_verts.size(); ++i)
	{
		m_visualBody.m_verts[i].m_color = Core::BOUNDING_VOLUME_COLOR;
	}

}

void Collision::DataStructures::BoundingBox::Update()
{
}

// void Collision::DataStructures::BoundingBox::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
// {
// 	if (!m_isVisible)
// 		return;
// 
// 	glm::vec3 size = glm::vec3(m_maxX - m_minX, m_maxY - m_minY, m_maxZ - m_minZ);
// 	glm::vec3 center = glm::vec3((m_maxX + m_minX) / 2, (m_maxY + m_minY) / 2, (m_maxZ + m_minZ) / 2);
// 	glm::mat4 modelMatrix = glm::translate(glm::mat4(1), center) * glm::scale(glm::mat4(1), size);
// 
// 	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * modelMatrix;
// 
// 	Rendering::ShapeRenderer::DrawWithLines(MVPMatrix, m_vao, m_indices, Rendering::BOUNDINGBOX);
// }

void Collision::DataStructures::BoundingBox::UpdateValues(glm::vec3 minCoords, glm::vec3 maxCoords)
{
	m_minX = minCoords.x;
	m_minY = minCoords.y;
	m_minZ = minCoords.z;

	m_maxX = maxCoords.x;
	m_maxY = maxCoords.y;
	m_maxZ = maxCoords.z;

	UpdateVisualVerts();
	

// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 0
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 1
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 2
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 3
// 
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 0
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 8
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 6
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 1
// 
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 2
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 5
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 6
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 8
// 
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 11
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 3
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 11
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 5
// 
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 6
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 1
// 	m_visualBody->m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 0
	

}

void Collision::DataStructures::BoundingBox::UpdateValues(std::vector<std::pair<glm::vec3, glm::vec3>> objectBounds)
{
	m_minX = objectBounds[0].first.x;
	m_minY = objectBounds[0].first.y;
	m_minZ = objectBounds[0].first.z;

	m_maxX = objectBounds[0].second.x;
	m_maxY = objectBounds[0].second.y;
	m_maxZ = objectBounds[0].second.z;

	for (int i = 1; i < objectBounds.size(); ++i)
	{
		if (objectBounds[i].first.x < m_minX)
			m_minX = objectBounds[i].first.x;
		if (objectBounds[i].first.y < m_minY)
			m_minY = objectBounds[i].first.y;
		if (objectBounds[i].first.z < m_minZ)
			m_minZ = objectBounds[i].first.z;

		if (objectBounds[i].second.x > m_maxX)
			m_maxX = objectBounds[i].second.x;
		if (objectBounds[i].second.y > m_maxY)
			m_maxY = objectBounds[i].second.y;
		if (objectBounds[i].second.z > m_maxZ)
			m_maxZ = objectBounds[i].second.z;
	}

	UpdateVisualVerts();
}

bool Collision::DataStructures::BoundingBox::Collides(const BoundingBox &other)
{
	return !(m_maxX < other.m_minX || m_minX > other.m_maxX || m_maxY < other.m_minY || m_minY > other.m_maxY || m_maxZ < other.m_minZ || m_minZ > other.m_maxZ);
}

void Collision::DataStructures::BoundingBox::UpdateVisualVerts()
{

	m_visualBody.m_verts.clear();
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 0
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 1
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 2
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 3

	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 4
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 5
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 6
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 7

	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 8
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 9
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 10
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 11

	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 12
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 13
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 14
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 15

	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 16
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 17
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 18
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_maxY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 19

	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 20
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_minZ), Core::BOUNDING_VOLUME_COLOR)); // 21
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_maxX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 22
	m_visualBody.m_verts.push_back(Rendering::VertexFormat(glm::vec3(m_minX, m_minY, m_maxZ), Core::BOUNDING_VOLUME_COLOR)); // 23

	glBindBuffer(GL_ARRAY_BUFFER, m_visualBody.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * m_visualBody.m_verts.size(), &m_visualBody.m_verts[0], GL_STATIC_DRAW);
}
