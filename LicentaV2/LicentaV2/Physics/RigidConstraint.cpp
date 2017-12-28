#include "RigidConstraint.h"

Physics::RigidConstraint::RigidConstraint(ClothNode *p, glm::vec3 qc, glm::vec3 nc, float stiffness, bool type)
{
	m_points.push_back(p);
	m_qc = qc;
	m_nc = nc;
	m_stiffness = stiffness;
	m_type = type;
}

void Physics::RigidConstraint::Init()
{
}

float Physics::RigidConstraint::ComputeConstraint()
{
	return glm::dot(m_points[0]->m_pos - m_qc, m_nc);
}

glm::vec3 Physics::RigidConstraint::ComputeDerivative(size_t index)
{
// 	if (index != 0)
// 	{
// 		
// 	}

	return glm::vec3(1);	
}

float Physics::RigidConstraint::ComputeScalingFactor()
{
	float deriv = glm::dot(glm::vec3(1), m_nc);
	return ComputeConstraint() / (m_points[0]->m_invMass * deriv * deriv);
}

void Physics::RigidConstraint::Solve(int iterationCount)
{
	if (ComputeConstraint() >= 0)
		return;

	float kp = 1 - std::powf(1 - m_stiffness, 1.f / iterationCount);
	glm::vec3 correction = -ComputeScalingFactor() * m_points[0]->m_invMass * m_nc * kp;

	m_points[0]->m_pos += correction;
}
