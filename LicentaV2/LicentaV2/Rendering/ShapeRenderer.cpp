#include "ShapeRenderer.h"
#include <iostream>

void Rendering::ShapeRenderer::CreateBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo, const std::vector<Rendering::VertexFormat> &verts, const std::vector<GLuint> &indices)
{
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * verts.size(), &verts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void *)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void *)(offsetof(Rendering::VertexFormat, Rendering::VertexFormat::m_color)));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void *)(offsetof(Rendering::VertexFormat, Rendering::VertexFormat::m_normal)));

	glBindVertexArray(0);
}

void Rendering::ShapeRenderer::Draw(const glm::mat4 viewProjection, const Rendering::VisualBody &body)
{
	//glUniform1i(2, collisionState);
 	//glUniform1i(2, 0);
 	glUniformMatrix4fv(3, 1, false, &viewProjection[0][0]);


// 	for (int i = 0; i < verts.size(); ++i)
// 	{
// 		glm::vec4 tmp = mvp * glm::vec4(verts[i].m_position, 1);
// 	//	std::cout << "verts " << verts[i].m_position.x << " " << verts[i].m_position.y << " " << verts[i].m_position.z << std::endl;
// 	//	std::cout << "tmp " << tmp.x << " " << tmp.y << " " << tmp.z << std::endl;
// 		verts[i].m_position = glm::vec3(tmp.x, tmp.y, tmp.z);
// 	}

	glBindBuffer(GL_ARRAY_BUFFER, body.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * body.m_verts.size(), &body.m_verts[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(body.m_vao);	

	glDrawElements(GL_TRIANGLES, (GLsizei)(body.m_indices.size()), GL_UNSIGNED_INT, &(body.m_indices[0]));
	glBindVertexArray(0);
}

void Rendering::ShapeRenderer::DrawWithLines(const glm::mat4 viewProjection, const GLuint vao, const std::vector<GLuint> indices, const int collisionState)
{

}

void Rendering::ShapeRenderer::DrawWithLines(const glm::mat4 viewProjection, const Rendering::VisualBody &body)
{	

	glUniformMatrix4fv(3, 1, false, &viewProjection[0][0]);

	glBindBuffer(GL_ARRAY_BUFFER, body.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * body.m_verts.size(), &body.m_verts[0], GL_STATIC_DRAW);

	glBindVertexArray(body.m_vao);

	glDrawElements(GL_LINE_LOOP, (GLsizei)body.m_indices.size(), GL_UNSIGNED_INT, &body.m_indices[0]);
	glBindVertexArray(0);	
}

void Rendering::ShapeRenderer::Draw(const glm::mat4 viewProjection, const GLuint vao, const std::vector<GLuint> indices, const int collisionState)
{

}
