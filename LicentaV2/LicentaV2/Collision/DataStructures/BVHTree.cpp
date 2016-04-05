#include "BVHTree.h"

using namespace Collision;
using namespace DataStructures;

Collision::DataStructures::BVHTree::BVHTree()
{
	m_type = DEFAULT;
}

Collision::DataStructures::BVHTree::~BVHTree()
{
	delete m_boundingBox;
	delete m_left;
	delete m_right;
}
