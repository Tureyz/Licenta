#include "CollisionTriangle.h"

#include "../../Core/Utils.hpp"
#include "../../Dependencies/glm/gtx/intersect.hpp"

void Collision::DataStructures::CollisionTriangle::Update()
{
	ComputeCenter();
	ResetCollisionState();

	ComputeFaceNormal();
	AddFaceNormalToVerts();
	ComputePlane();
}

bool Collision::DataStructures::CollisionTriangle::TriangleTest(CollisionTriangle &a, CollisionTriangle &b)
{
// 	if (a.PointsSameSide(b))
// 		return false;
// 	if (b.PointsSameSide(a))
// 		return false;

	if (a.PointsSameSideDet(b))
		return false;
	if (b.PointsSameSideDet(a))
		return false;	

	std::pair<glm::vec3, glm::vec3> lineIntersect = Plane::PlaneIntersect(a.m_plane, b.m_plane);

	glm::vec3 normalizedLineSeg = glm::normalize(lineIntersect.first + 15.f * lineIntersect.second);
	
	float dx = abs(glm::acos(glm::dot(normalizedLineSeg, glm::vec3(1, 0, 0))));
	float dy = abs(glm::acos(glm::dot(normalizedLineSeg, glm::vec3(0, 1, 0))));
	float dz = abs(glm::acos(glm::dot(normalizedLineSeg, glm::vec3(0, 0, 1))));

	std::pair<float, float> t1Proj, t2Proj;

	if (dx <= dy && dx <= dz)
	{
		t1Proj.first = a.m_verts[0]->m_position.x;
		t1Proj.second = a.m_verts[0]->m_position.x;

		for (int i = 1; i < a.m_verts.size(); ++i)
		{
			if (a.m_verts[i]->m_position.x < t1Proj.first)
				t1Proj.first = a.m_verts[i]->m_position.x;
			if (a.m_verts[i]->m_position.x > t1Proj.second)
				t1Proj.second = a.m_verts[i]->m_position.x;
		}

		t2Proj.first = b.m_verts[0]->m_position.x;
		t2Proj.second = b.m_verts[0]->m_position.x;

		for (int i = 1; i < b.m_verts.size(); ++i)
		{
			if (b.m_verts[i]->m_position.x < t2Proj.first)
				t2Proj.first = b.m_verts[i]->m_position.x;
			if (b.m_verts[i]->m_position.x > t2Proj.second)
				t2Proj.second = b.m_verts[i]->m_position.x;
		}
	}
	else if (dy <= dx && dy <= dz)
	{
		t1Proj.first = a.m_verts[0]->m_position.y;
		t1Proj.second = a.m_verts[0]->m_position.y;

		for (int i = 1; i < a.m_verts.size(); ++i)
		{
			if (a.m_verts[i]->m_position.y < t1Proj.first)
				t1Proj.first = a.m_verts[i]->m_position.y;
			if (a.m_verts[i]->m_position.y > t1Proj.second)
				t1Proj.second = a.m_verts[i]->m_position.y;
		}

		t2Proj.first = b.m_verts[0]->m_position.y;
		t2Proj.second = b.m_verts[0]->m_position.y;

		for (int i = 1; i < b.m_verts.size(); ++i)
		{
			if (b.m_verts[i]->m_position.y < t2Proj.first)
				t2Proj.first = b.m_verts[i]->m_position.y;
			if (b.m_verts[i]->m_position.y > t2Proj.second)
				t2Proj.second = b.m_verts[i]->m_position.y;
		}
	}

	else if (dz <= dx && dz <= dy)
	{
		t1Proj.first = a.m_verts[0]->m_position.z;
		t1Proj.second = a.m_verts[0]->m_position.z;

		for (int i = 1; i < a.m_verts.size(); ++i)
		{
			if (a.m_verts[i]->m_position.z < t1Proj.first)
				t1Proj.first = a.m_verts[i]->m_position.z;
			if (a.m_verts[i]->m_position.z > t1Proj.second)
				t1Proj.second = a.m_verts[i]->m_position.z;
		}

		t2Proj.first = b.m_verts[0]->m_position.z;
		t2Proj.second = b.m_verts[0]->m_position.z;

		for (int i = 1; i < b.m_verts.size(); ++i)
		{
			if (b.m_verts[i]->m_position.z < t2Proj.first)
				t2Proj.first = b.m_verts[i]->m_position.z;
			if (b.m_verts[i]->m_position.z > t2Proj.second)
				t2Proj.second = b.m_verts[i]->m_position.z;
		}
	}

// 	return (t1Proj.first < t2Proj.first && t1Proj.second > t2Proj.first) || (t1Proj.first < t2Proj.second && t1Proj.second > t2Proj.second) ||
// 		(t1Proj.first < t2Proj.first && t1Proj.second > t2Proj.second) || (t1Proj.first > t2Proj.first && t1Proj.second < t2Proj.second);


	return !(t1Proj.second <= t1Proj.first || t2Proj.second <= t1Proj.first);


	//210


}

glm::vec3 Collision::DataStructures::CollisionTriangle::GetCenter()
{
	return m_center;
}

void Collision::DataStructures::CollisionTriangle::SetColliding()
{
	m_collisionState = Rendering::CollisionState::COLLIDING;

	m_verts[0]->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_verts[1]->m_color = Core::COLLIDING_OBJECT_COLOR;
	m_verts[2]->m_color = Core::COLLIDING_OBJECT_COLOR;
}

void Collision::DataStructures::CollisionTriangle::ComputeFaceNormal()
{
	m_faceNormal = glm::cross(m_verts[1]->m_position - m_verts[0]->m_position, m_verts[2]->m_position - m_verts[0]->m_position);
}

void Collision::DataStructures::CollisionTriangle::ComputeCenter()
{
	m_center = (m_verts[0]->m_position + m_verts[1]->m_position + m_verts[2]->m_position) / 3.f;
}

void Collision::DataStructures::CollisionTriangle::ComputePlane()
{
	m_plane.m_normal = glm::normalize(m_faceNormal);
	m_plane.m_d = glm::dot(m_plane.m_normal, m_verts[0]->m_position);
}

glm::vec3 Collision::DataStructures::CollisionTriangle::Barycentric(glm::vec3 p)
{
	glm::vec3 result;

	// Unnormalized triangle normal

	// Nominators and one-over-denominator for u and v ratios
	float nu, nv, ood;
	// Absolute components for determining projection plane
	float x = abs(m_faceNormal.x), y = abs(m_faceNormal.y), z = abs(m_faceNormal.z);
	// Compute areas in plane of largest projection
	if (x >= y && x >= z)
	{
		// x is largest, project to the yz plane
		nu = TriArea2D(p.y, p.z, m_verts[1]->m_position.y, m_verts[1]->m_position.z, m_verts[2]->m_position.y, m_verts[2]->m_position.z); // Area of PBC in yz plane
		nv = TriArea2D(p.y, p.z, m_verts[2]->m_position.y, m_verts[2]->m_position.z, m_verts[0]->m_position.y, m_verts[0]->m_position.z); // Area of PCA in yz plane
		ood = 1.0f / m_faceNormal.x; // 1/(2*area of ABC in yz plane)
	}
	else if (y >= x && y >= z)
	{
		// y is largest, project to the xz plane
		nu = TriArea2D(p.x, p.z, m_verts[1]->m_position.x, m_verts[1]->m_position.z, m_verts[2]->m_position.x, m_verts[2]->m_position.z);
		nv = TriArea2D(p.x, p.z, m_verts[2]->m_position.x, m_verts[2]->m_position.z, m_verts[0]->m_position.x, m_verts[0]->m_position.z);
		ood = 1.0f / -m_faceNormal.y;
	}
	else
	{
		// z is largest, project to the xy plane
		nu = TriArea2D(p.x, p.y, m_verts[1]->m_position.x, m_verts[1]->m_position.y, m_verts[2]->m_position.x, m_verts[2]->m_position.y);
		nv = TriArea2D(p.x, p.y, m_verts[2]->m_position.x, m_verts[2]->m_position.y, m_verts[0]->m_position.x, m_verts[0]->m_position.y);
		ood = 1.0f / m_faceNormal.z;
	}
	float u = nu * ood;
	float v = nv * ood;
	float w = 1.0f - u - v;

	return glm::vec3(u, v, w);
}

void Collision::DataStructures::CollisionTriangle::ResetCollisionState()
{
	m_collisionState = Rendering::CollisionState::DEFAULT;
	m_verts[0]->m_color = Core::DEFAULT_OBJECT_COLOR;
	m_verts[1]->m_color = Core::DEFAULT_OBJECT_COLOR;
	m_verts[2]->m_color = Core::DEFAULT_OBJECT_COLOR;
}

void Collision::DataStructures::CollisionTriangle::AddFaceNormalToVerts()
{
	for (int i = 0; i < 3; ++i)
	{
		m_verts[i]->m_normal += m_faceNormal;
	}
}

bool Collision::DataStructures::CollisionTriangle::EdgeEdgeTest(Collision::DataStructures::Edge *edge1, Collision::DataStructures::Edge *edge2)
{


	return false;
}

bool Collision::DataStructures::CollisionTriangle::PointTriangleTest(glm::vec3 point, CollisionTriangle &triangle)
{
	glm::vec3 bar = triangle.Barycentric(point);
	return bar.y > 0.f && bar.z > 0.f && (bar.y + bar.z) <= 1.f;
}

float Collision::DataStructures::CollisionTriangle::TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
{
	return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
}

bool Collision::DataStructures::CollisionTriangle::PointsSameSide(const CollisionTriangle &other)
{
	glm::vec4 planeEc = glm::vec4(m_plane.m_normal, m_plane.m_d);

	float d1 = glm::dot(planeEc, glm::vec4(other.m_verts[0]->m_position, 1));
	float d2 = glm::dot(planeEc, glm::vec4(other.m_verts[1]->m_position, 1));
	float d3 = glm::dot(planeEc, glm::vec4(other.m_verts[2]->m_position, 1));

	return (d1 < 0 && d2 < 0 && d3 < 0) || (d1 > 0 && d2 > 0 && d3 > 0);
}

bool Collision::DataStructures::CollisionTriangle::PointsSameSideDet(const CollisionTriangle &other)
{
	float d1 = PlaneDeterminant(*this, other.m_verts[0]);
	float d2 = PlaneDeterminant(*this, other.m_verts[1]);
	float d3 = PlaneDeterminant(*this, other.m_verts[2]);

	return (d1 < 0 && d2 < 0 && d3 < 0) || (d1 > 0 && d2 > 0 && d3 > 0);
}

float Collision::DataStructures::CollisionTriangle::PlaneDeterminant(const CollisionTriangle &tri, const Rendering::VertexFormat *point)
{

	glm::vec3 e1 = tri.m_verts[1]->m_position - tri.m_verts[0]->m_position;
	glm::vec3 e2 = tri.m_verts[2]->m_position - tri.m_verts[0]->m_position;
	glm::vec3 e3 = point->m_position - tri.m_verts[0]->m_position;

	return glm::determinant(glm::mat3(e1, e2, e3));
}
