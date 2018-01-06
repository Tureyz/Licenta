#pragma once
#include "ContactData.h"
#include "../../Physics/ClothNode.h"
#include "../../Physics/ClothTriangle.h"

namespace Collision
{
	namespace DataStructures
	{
		class PointTriangleContactData : public ContactData
		{
		public:
			Physics::ClothNode *m_point;
			Physics::ClothTriangle *m_triangle;
			glm::vec3 m_barycentric;
			bool m_continuous;

			virtual void ColorNodes() override;

			virtual void UpdateProjections() override;

			virtual void UpdateAverageVels() override;

		};
	}
}

namespace std
{
	template <> struct hash<Collision::DataStructures::PointTriangleContactData>
	{
		inline size_t operator()(const Collision::DataStructures::PointTriangleContactData &v) const {
			std::hash<Physics::ClothNode *> hasher;
			std::hash<Physics::ClothTriangle *> hasher2;
			return hasher(v.m_point) ^ hasher2(v.m_triangle);
		}
	};

	template <> struct equal_to<Collision::DataStructures::PointTriangleContactData>
	{
		inline bool operator()(const Collision::DataStructures::PointTriangleContactData &l, const Collision::DataStructures::PointTriangleContactData &r) const
		{
			return l.m_point == r.m_point && l.m_triangle == r.m_triangle;
		}
	};
}