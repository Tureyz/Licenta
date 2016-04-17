#pragma once
#include <vector>
#include "../../Rendering/IPhysicsObject.h"

namespace Collision
{
	namespace DataStructures
	{
		enum SplittingAxisType { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

		class KDTreeNode
		{
		public:
			KDTreeNode();
			~KDTreeNode();
			KDTreeNode *m_left, *m_right;
			glm::vec3 m_minCoords;
			glm::vec3 m_maxCoords;
			SplittingAxisType m_axis;
			float m_splittingPoint;
			std::vector<Rendering::IPhysicsObject *> m_objects;
		private:
		};
	}
}