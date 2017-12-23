#pragma once
#include "../../Dependencies/glm/glm.hpp"
#include "../../Rendering/VertexFormat.h"

namespace Collision
{
	namespace DataStructures
	{
		class ClothNode
		{
		public:
			glm::vec3 m_pos;
			glm::vec3 m_vel;
			float m_mass;
			float m_invMass;
			Rendering::VertexFormat *m_vertexLink;

			ClothNode(glm::vec3 pos, glm::vec3 vel, float mass, Rendering::VertexFormat *parent) : m_pos(pos), m_vel(vel), m_mass(mass), m_invMass(1.f / mass), m_vertexLink(parent) {}

		};
	}
}
