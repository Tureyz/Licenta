#pragma once
#include "../Dependencies/glm/glm.hpp"
#include "../Rendering/VertexFormat.h"

namespace Physics
{
	class ClothNode
	{
	public:
		glm::vec3 m_pos;
		glm::vec3 m_vel;
		float m_mass;
		float m_invMass;
		Rendering::VertexFormat *m_vertexLink;
		size_t m_ID;

		ClothNode(glm::vec3 pos, glm::vec3 vel, Rendering::VertexFormat *parent, size_t ID) : m_pos(pos), m_vel(vel), m_vertexLink(parent), m_ID(ID) { m_mass = m_invMass = 0; }

	};
}
