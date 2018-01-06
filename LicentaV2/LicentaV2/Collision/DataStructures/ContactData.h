#pragma once
#include "../../Dependencies/glm/glm.hpp"

namespace Collision
{
	namespace DataStructures
	{
		class ContactData
		{
		public:
			glm::vec3 m_normal;
			float m_t;
			float m_h;

			virtual void ColorNodes() = 0;
			virtual void UpdateProjections() = 0;
			virtual void UpdateAverageVels() = 0;
		};
	}
}
