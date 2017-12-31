#pragma once

#include "Constraint.h"

namespace Physics
{
	class PointTriangleSelfConstraint : public Constraint
	{
	public:
		PointTriangleSelfConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type, float h);

		virtual void Init() override;

		virtual float ComputeConstraint() override;

		virtual glm::vec3 ComputeDerivative(size_t index) override;

		virtual float ComputeScalingFactor() override;

		virtual void Solve(int iterationCount) override;
	private:
		float m_h;

	};
}
