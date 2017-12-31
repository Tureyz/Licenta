#pragma once
#include <vector>
#include <functional>

#include "../Dependencies/glm/glm.hpp"
#include "ClothNode.h"


namespace Physics
{
	static const float EPS = 1.f / 1000000;

	class Constraint
	{
	public:
		//Constraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type);
		virtual void Init() = 0;
		virtual float ComputeConstraint() = 0;
		virtual glm::vec3 ComputeDerivative(size_t index) = 0;
		virtual float ComputeScalingFactor() = 0;

		virtual void Solve(int iterationCount) = 0;

		int m_cardinality;
		std::vector<Physics::ClothNode *> m_points;
		float m_stiffness;
		bool m_type;
		std::vector<float> m_auxValues;


	};
}
