#include "OctreeNode.h"

Collision::DataStructures::OctreeNode::OctreeNode()
{
	m_halfW = 0;
	m_center = glm::vec3(0);
	m_children.resize(8);
	for (int i = 0; i < m_children.size(); ++i)
	{
		m_children[i] = NULL;
	}
}

Collision::DataStructures::OctreeNode::~OctreeNode()
{
	for (int i = 0; i < m_children.size(); ++i)
	{
		delete m_children[i];
	}
}
