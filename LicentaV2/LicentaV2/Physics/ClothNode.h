#pragma once
#include "../Dependencies/glm/glm.hpp"
#include "../Rendering/VertexFormat.h"
#include "DisjointSet.h"

namespace Physics
{
	class ClothNode
	{
	public:
		glm::vec3 m_pos;
		glm::vec3 m_projection;
		glm::vec3 m_vel;
		glm::vec3 m_averageVel;
		//glm::vec3 m_impulseAcc;
		float m_mass;
		float m_invMass;
		Rendering::VertexFormat *m_vertexLink;
		size_t m_ID;
		bool m_isFixed;
		bool m_colliding;
		size_t m_impactZoneID;
		Physics::DisjointSet *m_dsNode;

		ClothNode(glm::vec3 pos, glm::vec3 vel, Rendering::VertexFormat *parent, size_t ID) : m_pos(pos), m_vel(vel), m_vertexLink(parent), m_ID(ID) { m_mass = m_invMass = 0; m_averageVel = m_vel; m_projection = m_pos; }

		glm::vec3 GetPositionInTime(const float t);

	};
}
