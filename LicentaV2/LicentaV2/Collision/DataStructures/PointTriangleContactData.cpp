#include "PointTriangleContactData.h"
#include "../../Core/Utils.hpp"

void Collision::DataStructures::PointTriangleContactData::ColorNodes()
{	
	m_point->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_triangle->m_nodes[0]->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_triangle->m_nodes[1]->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_triangle->m_nodes[2]->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
}

void Collision::DataStructures::PointTriangleContactData::UpdateProjections()
{
	m_triangle->m_nodes[0]->m_projection = m_triangle->m_nodes[0]->m_pos + m_triangle->m_nodes[0]->m_averageVel * Core::TIME_STEP;
	m_triangle->m_nodes[1]->m_projection = m_triangle->m_nodes[1]->m_pos + m_triangle->m_nodes[1]->m_averageVel * Core::TIME_STEP;
	m_triangle->m_nodes[2]->m_projection = m_triangle->m_nodes[2]->m_pos + m_triangle->m_nodes[2]->m_averageVel * Core::TIME_STEP;
	m_point->m_projection = m_point->m_pos + m_point->m_averageVel * Core::TIME_STEP;
}

void Collision::DataStructures::PointTriangleContactData::UpdateAverageVels()
{
	m_triangle->m_nodes[0]->m_averageVel = (m_triangle->m_nodes[0]->m_projection - m_triangle->m_nodes[0]->m_pos) / Core::TIME_STEP;
	m_triangle->m_nodes[1]->m_averageVel = (m_triangle->m_nodes[1]->m_projection - m_triangle->m_nodes[1]->m_pos) / Core::TIME_STEP;
	m_triangle->m_nodes[2]->m_averageVel = (m_triangle->m_nodes[2]->m_projection - m_triangle->m_nodes[2]->m_pos) / Core::TIME_STEP;
	m_point->m_averageVel = (m_point->m_projection - m_point->m_pos) / Core::TIME_STEP;
}
