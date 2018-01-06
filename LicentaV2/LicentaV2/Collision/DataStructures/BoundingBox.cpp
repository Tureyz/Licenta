#include "BoundingBox.h"
#include "../../Core/Utils.hpp"

Collision::DataStructures::BoundingBox::BoundingBox()
{
	SetVisible(false);
	m_visualBody = NULL;
	m_thickness = 0;
}

Collision::DataStructures::BoundingBox::~BoundingBox()
{
}

void Collision::DataStructures::BoundingBox::CreateVisualBody(Rendering::VisualBody *visualBody)
{
	m_visualBody = visualBody;


	for (int i = 0; i < m_visualBody->m_verts.size(); ++i)
	{
		m_visualBody->m_verts[i].m_color = Core::BOUNDING_VOLUME_COLOR;
	}

}

void Collision::DataStructures::BoundingBox::Update()
{
}


void Collision::DataStructures::BoundingBox::UpdateValuesUnsorted(glm::vec3 p1, glm::vec3 p2)
{
	glm::vec3 minCoords, maxCoords;

	if (p1.x < p2.x)
	{
		minCoords.x = p1.x;
		maxCoords.x = p2.x;
	}
	else
	{
		minCoords.x = p2.x;
		maxCoords.x = p1.x;
	}

	if (p1.y < p2.y)
	{
		minCoords.y = p1.y;
		maxCoords.y = p2.y;
	}
	else
	{
		minCoords.y = p2.y;
		maxCoords.y = p1.y;
	}

	if (p1.z < p2.z)
	{
		minCoords.z = p1.z;
		maxCoords.z = p2.z;
	}
	else
	{
		minCoords.z = p2.z;
		maxCoords.z = p1.z;
	}

	UpdateValues(minCoords, maxCoords);
}

void Collision::DataStructures::BoundingBox::UpdateValuesUnsorted(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4)
{
	glm::vec3 minCoords, maxCoords;

	minCoords.x = (p1.x < p2.x && p1.x < p3.x && p1.x < p4.x) ? p1.x : (p2.x < p3.x && p2.x < p4.x) ? p2.x : (p3.x < p4.x) ? p3.x : p4.x;
	minCoords.y = (p1.y < p2.y && p1.y < p3.y && p1.y < p4.y) ? p1.y : (p2.y < p3.y && p2.y < p4.y) ? p2.y : (p3.y < p4.y) ? p3.y : p4.y;
	minCoords.z = (p1.z < p2.z && p1.z < p3.z && p1.z < p4.z) ? p1.z : (p2.z < p3.z && p2.z < p4.z) ? p2.z : (p3.z < p4.z) ? p3.z : p4.z;

	maxCoords.x = (p1.x > p2.x && p1.x > p3.x && p1.x > p4.x) ? p1.x : (p2.x > p3.x && p2.x > p4.x) ? p2.x : (p3.x > p4.x) ? p3.x : p4.x;
	maxCoords.y = (p1.y > p2.y && p1.y > p3.y && p1.y > p4.y) ? p1.y : (p2.y > p3.y && p2.y > p4.y) ? p2.y : (p3.y > p4.y) ? p3.y : p4.y;
	maxCoords.z = (p1.z > p2.z && p1.z > p3.z && p1.z > p4.z) ? p1.z : (p2.z > p3.z && p2.z > p4.z) ? p2.z : (p3.z > p4.z) ? p3.z : p4.z;

	UpdateValues(minCoords, maxCoords);
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

	Enlarge();

	if (m_visualBody)
		UpdateVisualVerts();
}

void Collision::DataStructures::BoundingBox::UpdateValues(std::vector<glm::vec3> coords)
{
	m_minX = coords[0].x;
	m_minY = coords[0].y;
	m_minZ = coords[0].z;

	m_maxX = coords[0].x;
	m_maxY = coords[0].y;
	m_maxZ = coords[0].z;

	for (int i = 1; i < coords.size(); ++i)
	{
		if (coords[i].x < m_minX)
			m_minX = coords[i].x;
		if (coords[i].y < m_minY)
			m_minY = coords[i].y;
		if (coords[i].z < m_minZ)
			m_minZ = coords[i].z;

		if (coords[i].x > m_maxX)
			m_maxX = coords[i].x;
		if (coords[i].y > m_maxY)
			m_maxY = coords[i].y;
		if (coords[i].z > m_maxZ)
			m_maxZ = coords[i].z;
	}

	Enlarge();

	if (m_visualBody)
		UpdateVisualVerts();
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

	Enlarge();

	if (m_visualBody)
		UpdateVisualVerts();
}

bool Collision::DataStructures::BoundingBox::Collides(const BoundingBox &other) const
{
	return !(m_maxX < other.m_minX || m_minX > other.m_maxX || m_maxY < other.m_minY || m_minY > other.m_maxY || m_maxZ < other.m_minZ || m_minZ > other.m_maxZ);
}

void Collision::DataStructures::BoundingBox::Enlarge()
{
	if (m_thickness > 0)
	{
		m_minX -= m_thickness;
		m_minY -= m_thickness;
		m_minZ -= m_thickness;

		m_maxX += m_thickness;
		m_maxY += m_thickness;
		m_maxZ += m_thickness;
	}
}

void Collision::DataStructures::BoundingBox::UpdateVisualVerts()
{

	m_visualBody->m_verts[0].m_position = glm::vec3(m_minX, m_minY, m_maxZ); // 0
	m_visualBody->m_verts[1].m_position = glm::vec3(m_maxX, m_minY, m_maxZ); // 1
	m_visualBody->m_verts[2].m_position = glm::vec3(m_maxX, m_maxY, m_maxZ); // 2
	m_visualBody->m_verts[3].m_position = glm::vec3(m_minX, m_maxY, m_maxZ); // 3
	m_visualBody->m_verts[4].m_position = glm::vec3(m_maxX, m_maxY, m_maxZ); // 4
	m_visualBody->m_verts[5].m_position = glm::vec3(m_maxX, m_maxY, m_minZ); // 5
	m_visualBody->m_verts[6].m_position = glm::vec3(m_maxX, m_minY, m_minZ); // 6
	m_visualBody->m_verts[7].m_position = glm::vec3(m_maxX, m_minY, m_maxZ); // 7
	m_visualBody->m_verts[8].m_position = glm::vec3(m_minX, m_minY, m_minZ); // 8
	m_visualBody->m_verts[9].m_position = glm::vec3(m_maxX, m_minY, m_minZ); // 9
	m_visualBody->m_verts[10].m_position = glm::vec3(m_maxX, m_maxY, m_minZ); // 10
	m_visualBody->m_verts[11].m_position = glm::vec3(m_minX, m_maxY, m_minZ); // 11
	m_visualBody->m_verts[12].m_position = glm::vec3(m_minX, m_minY, m_minZ); // 12
	m_visualBody->m_verts[13].m_position = glm::vec3(m_minX, m_minY, m_maxZ); // 13
	m_visualBody->m_verts[14].m_position = glm::vec3(m_minX, m_maxY, m_maxZ); // 14
	m_visualBody->m_verts[15].m_position = glm::vec3(m_minX, m_maxY, m_minZ); // 15
	m_visualBody->m_verts[16].m_position = glm::vec3(m_maxX, m_maxY, m_maxZ); // 16
	m_visualBody->m_verts[17].m_position = glm::vec3(m_minX, m_maxY, m_maxZ); // 17
	m_visualBody->m_verts[18].m_position = glm::vec3(m_minX, m_maxY, m_minZ); // 18
	m_visualBody->m_verts[19].m_position = glm::vec3(m_maxX, m_maxY, m_minZ); // 19
	m_visualBody->m_verts[20].m_position = glm::vec3(m_minX, m_minY, m_minZ); // 20
	m_visualBody->m_verts[21].m_position = glm::vec3(m_maxX, m_minY, m_minZ); // 21
	m_visualBody->m_verts[22].m_position = glm::vec3(m_maxX, m_minY, m_maxZ); // 22
	m_visualBody->m_verts[23].m_position = glm::vec3(m_minX, m_minY, m_maxZ); // 23

	glBindBuffer(GL_ARRAY_BUFFER, m_visualBody->m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * m_visualBody->m_verts.size(), &m_visualBody->m_verts[0], GL_STATIC_DRAW);
}
