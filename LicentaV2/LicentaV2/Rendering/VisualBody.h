#pragma once
#include <vector>

#include "VertexFormat.h"
#include "..\Dependencies\glew\glew.h"

namespace Rendering
{
	class VisualBody
	{
	public:
	
		VisualBody(std::vector<VertexFormat> verts, std::vector<GLuint> indices, GLuint vao, GLuint vbo, GLuint ibo);
		VisualBody(GLuint vao, GLuint vbo, GLuint ibo);
		VisualBody();
		std::pair<glm::vec3, glm::vec3> UpdateVerts(glm::mat4 &modelMat);
		
		//std::vector<VertexFormat> m_initialVerts;
		std::vector<VertexFormat> m_verts;
		std::vector<GLuint> m_indices;

		GLuint m_vao, m_vbo, m_ibo;
	};
}
