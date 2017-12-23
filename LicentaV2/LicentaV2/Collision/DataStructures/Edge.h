#pragma once

#include "../../Rendering/VertexFormat.h"
#include <unordered_set>


namespace Collision
{
	namespace DataStructures
	{
		class Edge
		{
		public:

			Edge(Rendering::VertexFormat *v1, Rendering::VertexFormat *v2, size_t id) : m_v1(v1), m_v2(v2), m_id(id) {}
			Rendering::VertexFormat *m_v1, *m_v2;
			std::unordered_set<size_t> m_triangles;

			size_t GetID() const { return m_id; }
			void SetID(size_t val) { m_id = val; }
		private:
			size_t m_id;
		};
	}
}

namespace std
{
	template <> struct equal_to<Collision::DataStructures::Edge>
	{
		inline bool operator()(const Collision::DataStructures::Edge &l, const Collision::DataStructures::Edge &r) const
		{
			return (l.m_v1 == r.m_v1 && l.m_v2 == r.m_v2) || (l.m_v1 == r.m_v2 && l.m_v2 == r.m_v1);
		}
	};

	template <> struct hash<Collision::DataStructures::Edge>
	{
		inline size_t operator()(const Collision::DataStructures::Edge &v) const {
			std::hash<glm::vec3> hasher;
			return hasher(v.m_v1->m_position) ^ hasher(v.m_v2->m_position);
		}
	};
}
