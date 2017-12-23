#pragma once
#include "../../Dependencies/glm/glm.hpp"
#include <unordered_set>


namespace Collision
{
	namespace DataStructures
	{
		struct Vec3EqualEps
		{
		public:
			//const float m_eps = 0.0001f;
			bool operator()(const glm::vec3 &a, const glm::vec3 &b) const
			{
				//return abs(a.x - b.x) < m_eps && abs(a.y - b.y) < m_eps && abs(a.z - b.z) < m_eps;
				return a.x == b.x && a.y == b.y && a.z == b.z;
			}
		};

		struct Vec3Hash
		{
		public:
			size_t operator()(const glm::vec3 &a) const
			{
				return std::hash<float>()(a.x) ^ std::hash<float>()(a.y) ^ std::hash<float>()(a.z);
			}
		};


		const float eps = 0.0001f;

		class Plane
		{
		public:
			Plane(glm::vec3 normal, float d) : m_normal(normal), m_d(d) {};
			Plane();

			float DistFromPlane(const glm::vec3 point);
			void GetSegmentPlaneIntersection(const glm::vec3 P1, const glm::vec3 P2, std::unordered_set<glm::vec3, Vec3Hash, Vec3EqualEps> &outSegTips);
			std::vector<glm::vec3> TrianglePlaneIntersection(const glm::vec3 triA, const glm::vec3 triB, const glm::vec3 triC);

			static std::pair<glm::vec3, glm::vec3> PlaneIntersect(const Plane &p1, const Plane &p2);
			glm::vec3 m_normal;
			float m_d;
		};
	}
}
