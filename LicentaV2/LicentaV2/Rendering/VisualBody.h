#pragma once
#include <vector>

#include "VertexFormat.h"
#include "..\Dependencies\glew\glew.h"

namespace Rendering
{
	enum VisualBodyType { OBJ_CUBE = 0, OBJ_SPHERE = 1, OBJ_TETRAHEDRON = 2, OBJ_CYLINDER = 3, OBJ_CONE = 4, OBJ_MESH = 5, OBJ_LINE_CUBE = 6, OBJ_RANDOM, OBJ_NUM_TOTAL = 7 };

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
