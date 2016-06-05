#include "ShapeRenderer.h"

void Rendering::ShapeRenderer::CreateBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo, std::vector<Rendering::VertexFormat> &verts, std::vector<GLuint> &indices)
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
	glBindVertexArray(0);
}

void Rendering::ShapeRenderer::Draw(const glm::mat4 mvp, const GLuint vao, const std::vector<GLuint> indices, const int collisionState)
{
	glUniform1i(2, collisionState);
	glUniformMatrix4fv(3, 1, false, &mvp[0][0]);
	glBindVertexArray(vao);

	glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, &indices[0]);
}
