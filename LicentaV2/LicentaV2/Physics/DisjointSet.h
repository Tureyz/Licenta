#pragma once
#include <vector>

namespace Physics
{
	class DisjointSet
	{
	public:

		DisjointSet();
		// std::vector<T> GetAllChildren();
		static DisjointSet *Find(DisjointSet *x);
		static void Union(DisjointSet *x, DisjointSet *y);

		DisjointSet *m_parent;
		float m_rank;
		//std::vector<DisjointSet<T> *> m_children;
	};

}
