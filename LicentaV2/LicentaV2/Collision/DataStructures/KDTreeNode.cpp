#include "KDTreeNode.h"

Collision::DataStructures::KDTreeNode::KDTreeNode()
{
	m_left = NULL;
	m_right = NULL;
	m_axis = AXIS_X;
	m_splittingPoint = 0.f;
}

Collision::DataStructures::KDTreeNode::~KDTreeNode()
{
	delete m_left;
	delete m_right;
}
