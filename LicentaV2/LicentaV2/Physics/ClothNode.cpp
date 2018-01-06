#include "ClothNode.h"

glm::vec3 Physics::ClothNode::GetPositionInTime(const float t)
{
	//return m_pos + (m_projection - m_pos) * t;

	return m_pos + m_averageVel * t;
}
