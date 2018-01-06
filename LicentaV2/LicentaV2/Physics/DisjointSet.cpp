#include "DisjointSet.h"


Physics::DisjointSet::DisjointSet()
{
	m_parent = this;
	m_rank = 0;
}

Physics::DisjointSet * Physics::DisjointSet::Find(DisjointSet *x)
{
	if (x->m_parent != x)
	{
		x->m_parent = Find(x->m_parent);
	}
	return x->m_parent;
}

void Physics::DisjointSet::Union(DisjointSet *x, DisjointSet *y)
{
	DisjointSet *xRoot = Find(x);
	DisjointSet *yRoot = Find(y);

	if (xRoot == yRoot)
		return;

	if (xRoot->m_rank < yRoot->m_rank)
	{
		xRoot->m_parent = yRoot;
		//yRoot->m_children.push_back(xRoot);
	}
	else if (xRoot->m_rank > yRoot->m_rank)
	{
		yRoot->m_parent = xRoot;
		//xRoot->m_children.push_back(yRoot);
	}
	else
	{
		yRoot->m_parent = xRoot;
	//	xRoot->m_children.push_back(yRoot);
		xRoot->m_rank++;
	}
}

// template<typename T>
// std::vector<T> Physics::DisjointSet<T>::GetAllChildren()
// {
// 	static std::vector<T> result;
// 
// // 	for (auto child : m_children)
// // 	{
// // 		result.push_back(child->m_value);
// // 		child->GetAllChildren();
// // 	}
// 
// 	return result;
// }
