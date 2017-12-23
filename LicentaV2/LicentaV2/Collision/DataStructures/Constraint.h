#pragma once
#include <vector>
#include "../../Dependencies/glm/glm.hpp"
#include "ClothNode.h"


namespace Collision
{
	namespace DataStructures
	{
		class Constraint
		{
		public:
			enum FunctionTypes { EDGE_STRETCH, TRIANGLE_BEND };
			int m_cardinality;
			std::vector<Collision::DataStructures::ClothNode *> m_points;
			float m_stiffness;
			bool m_type;
			FunctionTypes m_funcType;
			std::vector<float> m_auxValues;


			static float StretchFunc(glm::vec3 p1, glm::vec3 p2, float l0);
			static float BendingFunc(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, float fi0);
			
		};
	}
}
