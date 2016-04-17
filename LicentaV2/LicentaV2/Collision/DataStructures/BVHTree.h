#pragma once

#include "../../Collision/DataStructures/BoundingBox.h"
#include "../../Rendering/IPhysicsObject.h"

namespace Collision
{
	namespace DataStructures
	{
		class BVHTree
		{
		public:
			enum NodeType {DEFAULT, NODE, LEAF};
			BVHTree();
			~BVHTree();
			BVHTree *m_left, *m_right;
			Collision::DataStructures::BoundingBox *m_boundingBox;
			Rendering::IPhysicsObject ** m_objects;
			size_t m_numObjects;
			NodeType m_type;
		private:
		};
	}
}