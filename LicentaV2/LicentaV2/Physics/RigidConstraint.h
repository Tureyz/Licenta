#pragma once

#include "Constraint.h"

namespace Physics
{
	class RigidConstraint : public Constraint
	{
	public:
		RigidConstraint(ClothNode *p, glm::vec3 qc, glm::vec3 nc, float stiffness, bool type);

		virtual void Init() override;

		virtual float ComputeConstraint() override;

		virtual glm::vec3 ComputeDerivative(size_t index) override;

		virtual float ComputeScalingFactor() override;

		virtual void Solve(int iterationCount) override;
	private:

		glm::vec3 m_qc;
		glm::vec3 m_nc;

	};
}
