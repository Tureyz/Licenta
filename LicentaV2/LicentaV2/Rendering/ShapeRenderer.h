#pragma once
#include <vector>

#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"

#include "VisualBody.h"


namespace Rendering
{
	class ShapeRenderer
	{
	public:
		static void CreateBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo, const std::vector<Rendering::VertexFormat> &verts, std::vector<GLuint> const &indices);
		static void Draw(const glm::mat4 mvp, const Rendering::VisualBody &visualBody);
		static void DrawWithLines(const glm::mat4 mvp, const Rendering::VisualBody &visualBody);
		static void Draw(const glm::mat4 mvp, const GLuint vao, const std::vector<GLuint> indices, const int collisionState);
		static void DrawWithLines(const glm::mat4 mvp, const GLuint vao, const std::vector<GLuint> indices, const  int collisionState);
	};
}