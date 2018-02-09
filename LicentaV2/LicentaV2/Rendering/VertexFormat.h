#pragma once
#ifndef  VertexFormat_H_
#define VertexFormat_H_

#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/glm/gtc/matrix_transform.hpp"
#include "../Dependencies/glm/gtx/transform.hpp"

namespace std
{
	template <> struct hash<glm::vec3>
	{
		inline size_t operator()(const glm::vec3 &v) const {
			std::hash<size_t> hasher;
			return hasher((uint64_t)(v.x)) ^ hasher((uint64_t)(v.y)) ^ hasher((uint64_t)(v.z));
		}
	};
}

namespace Rendering
{
	enum CollisionState { DEFAULT = 0, COLLIDING = 1, ACTIVE = 2, BOUNDINGBOX = 3, COLLISIONMETHOD = 4 };

	struct VertexFormat
	{
		glm::vec3 m_position;
		glm::vec4 m_color;
		glm::vec3 m_normal;
		glm::vec2 m_uv;

		VertexFormat()
		{
			m_position = glm::vec3(0);
			m_color = glm::vec4(1);
			m_normal = glm::vec3(0);
			m_uv = glm::vec2(0);
		}

		VertexFormat(const glm::vec3 &pos, const glm::vec4 &color)
		{
			m_position = pos;
			m_color = color;
			m_normal = glm::vec3(0);
			m_uv = glm::vec2(0);
		}

		VertexFormat(const glm::vec3 &pos, const glm::vec4 &color, const glm::vec2 &uv)
		{
			m_position = pos;
			m_color = color;
			m_normal = glm::vec3(0);
			m_uv = uv;
		}
	};
}

#endif