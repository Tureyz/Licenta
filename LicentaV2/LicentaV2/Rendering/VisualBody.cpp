#include "VisualBody.h"

Rendering::VisualBody::VisualBody(std::vector<VertexFormat> verts, std::vector<GLuint> indices, GLuint vao, GLuint vbo, GLuint ibo)
{
	//m_initialVerts = verts;
	m_verts = verts;
	m_indices = indices;
	m_vao = vao;
	m_vbo = vbo;
	m_ibo = ibo;
}

Rendering::VisualBody::VisualBody()
{

}

Rendering::VisualBody::VisualBody(GLuint vao, GLuint vbo, GLuint ibo)
{
	m_vao = vao;
	m_vbo = vbo;
	m_ibo = ibo;
}

std::pair<glm::vec3, glm::vec3> Rendering::VisualBody::UpdateVerts(glm::mat4 &modelMat)
{

	glm::vec4 tmp = modelMat * glm::vec4(m_verts[0].m_position, 1);
	m_verts[0].m_position = glm::vec3(tmp.x, tmp.y, tmp.z);
	glm::vec3 minCoords, maxCoords;

	minCoords = maxCoords = m_verts[0].m_position;

	for (int i = 1; i < m_verts.size(); ++i)
	{
		tmp = modelMat * glm::vec4(m_verts[i].m_position, 1);
		m_verts[i].m_position = glm::vec3(tmp.x, tmp.y, tmp.z);

		if (tmp.x < minCoords.x)
			minCoords.x = tmp.x;
		if (tmp.y < minCoords.y)
			minCoords.y = tmp.y;
		if (tmp.z < minCoords.z)
			minCoords.z = tmp.z;

		if (tmp.x > maxCoords.x)
			maxCoords.x = tmp.x;
		if (tmp.y > maxCoords.y)
			maxCoords.y = tmp.y;
		if (tmp.z > maxCoords.z)
			maxCoords.z = tmp.z;
	}

// 	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
// 	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * m_verts.size(), &m_verts[0], GL_STATIC_DRAW);

	return std::make_pair(minCoords, maxCoords);
}
