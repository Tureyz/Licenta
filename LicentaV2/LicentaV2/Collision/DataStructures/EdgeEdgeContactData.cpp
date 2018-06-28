#include "EdgeEdgeContactData.h"
#include "../../Core/Utils.hpp"

void Collision::DataStructures::EdgeEdgeContactData::ColorNodes()
{
	m_e1->m_v1->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_e1->m_v2->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_e2->m_v1->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_e2->m_v2->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
}

void Collision::DataStructures::EdgeEdgeContactData::UpdateProjections()
{
	m_e1->m_v1->m_projection = m_e1->m_v1->m_pos + m_e1->m_v1->m_averageVel * Core::PHYSICS_TIME_STEP;
	m_e1->m_v2->m_projection = m_e1->m_v2->m_pos + m_e1->m_v2->m_averageVel * Core::PHYSICS_TIME_STEP;
	m_e2->m_v1->m_projection = m_e2->m_v1->m_pos + m_e2->m_v1->m_averageVel * Core::PHYSICS_TIME_STEP;
	m_e2->m_v2->m_projection = m_e2->m_v2->m_pos + m_e2->m_v2->m_averageVel * Core::PHYSICS_TIME_STEP;
}

void Collision::DataStructures::EdgeEdgeContactData::UpdateAverageVels()
{
	m_e1->m_v1->m_averageVel = (m_e1->m_v1->m_projection - m_e1->m_v1->m_pos) / Core::PHYSICS_TIME_STEP;
	m_e1->m_v2->m_averageVel = (m_e1->m_v2->m_projection - m_e1->m_v2->m_pos) / Core::PHYSICS_TIME_STEP;
	m_e2->m_v1->m_averageVel = (m_e2->m_v1->m_projection - m_e2->m_v1->m_pos) / Core::PHYSICS_TIME_STEP;
	m_e2->m_v2->m_averageVel = (m_e2->m_v2->m_projection - m_e2->m_v2->m_pos) / Core::PHYSICS_TIME_STEP;
}
