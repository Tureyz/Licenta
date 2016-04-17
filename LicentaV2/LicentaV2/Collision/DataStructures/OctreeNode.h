#pragma once
#include "../../Dependencies/glm/glm.hpp"
#include <vector>
#include "../../Rendering/IPhysicsObject.h"

namespace Collision
{
	namespace DataStructures
	{
		class OctreeNode
		{
		public:
			OctreeNode();
			~OctreeNode();
			float m_halfW;
			glm::vec3 m_center;
			std::vector<OctreeNode *> m_children;
			std::vector<Rendering::IPhysicsObject *> m_objects;
		private:

		};
	}
}