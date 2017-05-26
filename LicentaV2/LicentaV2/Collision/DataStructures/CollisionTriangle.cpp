#include "CollisionTriangle.h"

#include "../../Core/Utils.hpp"
void Collision::DataStructures::CollisionTriangle::Update()
{
	m_center = (m_transformedVerts[0]->m_position + m_transformedVerts[1]->m_position + m_transformedVerts[2]->m_position) / 3.f;
	m_collisionState = Rendering::CollisionState::DEFAULT;
	m_initialVerts[0]->m_color = Core::DEFAULT_OBJECT_COLOR;
	m_initialVerts[1]->m_color = Core::DEFAULT_OBJECT_COLOR;
	m_initialVerts[2]->m_color = Core::DEFAULT_OBJECT_COLOR;
}

bool Collision::DataStructures::CollisionTriangle::TriangleTest(CollisionTriangle &a, CollisionTriangle &b)
{
	return true;

	if (a.PointInTriangleTests(b))
		return true;
	if (b.PointInTriangleTests(a))
		return true;

	return EdgeEdgeTests(a, b);	
}

glm::vec3 Collision::DataStructures::CollisionTriangle::GetCenter()
{	
	return m_center;
}

void Collision::DataStructures::CollisionTriangle::SetColliding()
{
	m_collisionState = Rendering::CollisionState::COLLIDING;

	m_initialVerts[0]->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_initialVerts[1]->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_initialVerts[2]->m_color = Core::COLLIDING_OBJECT_COLOR;
}

bool Collision::DataStructures::CollisionTriangle::PointInTriangleTests(CollisionTriangle &other)
{
	for (Rendering::VertexFormat *point : this->m_transformedVerts)
	{
		if (PointTriangleTest(point, other))
			return true;
	}

	return false;
}

bool Collision::DataStructures::CollisionTriangle::EdgeEdgeTests(CollisionTriangle &a, CollisionTriangle &b)
{
	
	for (auto e1 : a.m_edges)
	{
		for (auto e2 : b.m_edges)
		{
			if (EdgeEdgeTest(e1, e2))
				return true;
		}
	}

	return false;
}

bool Collision::DataStructures::CollisionTriangle::EdgeEdgeTest(std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> edge1, std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> edge2)
{


	return false;
}

bool Collision::DataStructures::CollisionTriangle::PointTriangleTest(Rendering::VertexFormat *point, CollisionTriangle &triangle)
{

	return false;
}
