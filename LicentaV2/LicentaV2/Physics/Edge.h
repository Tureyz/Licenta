#pragma once
#include <unordered_set>

#include "ClothNode.h"


namespace Physics
{
	class Edge
	{
	public:

		Edge(Physics::ClothNode *v1, Physics::ClothNode *v2, size_t id) : m_v1(v1), m_v2(v2), m_id(id) {}
		Physics::ClothNode *m_v1, *m_v2;
		mutable std::unordered_set<size_t> m_triangles;

		size_t GetID() const { return m_id; }
		void SetID(size_t val) { m_id = val; }
	private:
		size_t m_id;
	};
}

namespace std
{
	template <> struct equal_to<Physics::Edge>
	{
		inline bool operator()(const Physics::Edge &l, const Physics::Edge &r) const
		{
			return (l.m_v1->m_pos == r.m_v1->m_pos && l.m_v2->m_pos == r.m_v2->m_pos) || (l.m_v1->m_pos == r.m_v2->m_pos && l.m_v2->m_pos == r.m_v1->m_pos);
		}
	};

	template <> struct hash<Physics::Edge>
	{
		inline size_t operator()(const Physics::Edge &v) const {
			std::hash<glm::vec3> hasher;
			return hasher(v.m_v1->m_pos) ^ hasher(v.m_v2->m_pos);
		}
	};
}
