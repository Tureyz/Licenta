#pragma once
#include <vector>

#include "../../Dependencies/glm/glm.hpp"
#include "../../Rendering/SceneObject.h"

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
			std::vector<Rendering::SceneObject *> m_objects;
		private:

		};
	}
}