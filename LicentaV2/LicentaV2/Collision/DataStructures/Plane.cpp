#include "Plane.h"

Collision::DataStructures::Plane::Plane()
{
	m_normal = glm::vec3(0);
	m_d = 0;
}

float Collision::DataStructures::Plane::DistFromPlane(glm::vec3 point)
{
	return glm::dot(m_normal, point) + m_d;
}

void Collision::DataStructures::Plane::GetSegmentPlaneIntersection(const glm::vec3 P1, const glm::vec3 P2, std::unordered_set<glm::vec3, Vec3Hash, Vec3EqualEps> &outSegTips)
{
	float d1 = DistFromPlane(P1), d2 = DistFromPlane(P2);

	bool bP1OnPlane = (abs(d1) < eps), bP2OnPlane = (abs(d2) < eps);

	if (bP1OnPlane)
		outSegTips.insert(P1);

	if (bP2OnPlane)
		outSegTips.insert(P2);

	if (bP1OnPlane && bP2OnPlane)
		return;

	if (d1 * d2 > eps)
		return;

	float t = d1 / (d1 - d2);
	outSegTips.insert(P1 + t * (P2 - P1));
}

std::vector<glm::vec3> Collision::DataStructures::Plane::TrianglePlaneIntersection(const glm::vec3 triA, const glm::vec3 triB, const glm::vec3 triC)
{
	std::unordered_set<glm::vec3, Vec3Hash, Vec3EqualEps> outSegTips;

	GetSegmentPlaneIntersection(triA, triB, outSegTips);
	GetSegmentPlaneIntersection(triB, triC, outSegTips);
	GetSegmentPlaneIntersection(triC, triA, outSegTips);

	return std::vector<glm::vec3>(outSegTips.begin(), outSegTips.end());
}

std::pair<glm::vec3, glm::vec3> Collision::DataStructures::Plane::PlaneIntersect(const Plane &p1, const Plane &p2)
{
	// Compute direction of intersection line
	glm::vec3 d = glm::cross(p1.m_normal, p2.m_normal);

	// If d is (near) zero, the planes are parallel (and separated)
	// or coincident, so they’re not considered intersecting
	float denom = glm::dot(d, d);
	if (denom < eps) return std::make_pair(glm::vec3(0), glm::vec3(0));

	// Compute point on intersection line	
	return std::make_pair(glm::cross(p1.m_d*p2.m_normal - p2.m_d*p1.m_normal, d) / denom, d);
}

