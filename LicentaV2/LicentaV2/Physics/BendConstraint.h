#pragma once

#include "Constraint.h"
#include "CollisionTriangle.h"

namespace Physics
{
	class BendConstraint : public Constraint
	{

	public:
		BendConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type);
		virtual void Init() override;

		virtual float ComputeConstraint() override;

		virtual glm::vec3 ComputeDerivative(size_t index) override;

		virtual float ComputeScalingFactor() override;

		virtual void Solve(int iterationCount) override;

	private:
		float m_initialAngle;

	};
}