#pragma once
#include "Constraint.h"

namespace Physics
{
	class StretchConstraint : public Constraint
	{
	public:
		StretchConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type);

		virtual void Init() override;

		virtual float ComputeConstraint() override;
		
	private:

		glm::vec3 DerivP1();
		glm::vec3 DerivP2();

		virtual glm::vec3 ComputeDerivative(size_t index) override;

		virtual float ComputeScalingFactor() override;

		virtual void Solve(int iterationCount) override;

		float m_l0;

	};
}
