#pragma once

#include <vector>

#include "ClothNode.h"
#include "Constraint.h"
#include "CollisionData.h"

namespace std
{

	
}

namespace Collision
{
	namespace DataStructures
	{
		class ClothBehavior
		{
		public:
			ClothBehavior(Collision::DataStructures::CollisionData *data);
			std::vector<ClothNode> m_nodes;
			std::vector<Constraint> m_constraints;
			float m_density;
		};
	}
}
