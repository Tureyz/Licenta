#pragma once

#include "ContactData.h"
#include "../../Physics/Edge.h"

namespace Collision
{
	namespace DataStructures
	{
		class EdgeEdgeContactData : public ContactData
		{
		public:
			float m_a, m_b;
			Physics::Edge *m_e1;
			Physics::Edge *m_e2;
			bool m_continuous;

			virtual void ColorNodes() override;

			virtual void UpdateProjections() override;

			virtual void UpdateAverageVels() override;

		};
	}
}

namespace std
{
	template <> struct hash<Collision::DataStructures::EdgeEdgeContactData>
	{
		inline size_t operator()(const Collision::DataStructures::EdgeEdgeContactData &v) const {
			std::hash<Physics::Edge *> hasher;			
			return hasher(v.m_e1) ^ hasher(v.m_e2);
		}
	};

	template <> struct equal_to<Collision::DataStructures::EdgeEdgeContactData>
	{
		inline bool operator()(const Collision::DataStructures::EdgeEdgeContactData &l, const Collision::DataStructures::EdgeEdgeContactData &r) const
		{
			return (l.m_e1 == r.m_e1 && l.m_e2 == r.m_e2) || (l.m_e1 == r.m_e2 && l.m_e2 == r.m_e1);
		}
	};
}