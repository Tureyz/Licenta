#pragma once

#include "CollisionTriangle.h"
#include "ClothNode.h"

namespace Physics
{
	class ClothTriangle
	{
	public:
		Physics::CollisionTriangle *m_triLink;
		std::vector<Physics::ClothNode *> m_nodes;

	};
}
