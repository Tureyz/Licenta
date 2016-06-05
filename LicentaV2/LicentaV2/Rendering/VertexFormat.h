#pragma once
#ifndef  VertexFormat_H_
#define VertexFormat_H_

#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/glm/gtc/matrix_transform.hpp"
#include "../Dependencies/glm/gtx/transform.hpp"

namespace Rendering
{
	struct VertexFormat
	{
		glm::vec3 m_position;
		glm::vec4 m_color;

		VertexFormat()
		{
			m_position = glm::vec3(0);
			m_color = glm::vec4(1);
		}

		VertexFormat(const glm::vec3 &pos, const glm::vec4 &color)
		{
			m_position = pos;
			m_color = color;
		}
	};
}

#endif