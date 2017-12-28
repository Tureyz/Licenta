#include "StretchConstraint.h"
#include <iostream>

Physics::StretchConstraint::StretchConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type)
{
	m_points = points;
	m_stiffness = stiffness;
	m_type = type;
}

void Physics::StretchConstraint::Init()
{
	m_l0 = glm::length(m_points[0]->m_pos - m_points[1]->m_pos);
}

float Physics::StretchConstraint::ComputeConstraint()
{
	return glm::length(m_points[0]->m_pos - m_points[1]->m_pos) - m_l0;
}

glm::vec3 Physics::StretchConstraint::DerivP1()
{
	glm::vec3 diff = m_points[0]->m_pos - m_points[1]->m_pos;

	return diff / length(diff);
}

glm::vec3 Physics::StretchConstraint::DerivP2()
{
	glm::vec3 diff = m_points[0]->m_pos - m_points[1]->m_pos;
	return -diff / length(diff);
}

glm::vec3 Physics::StretchConstraint::ComputeDerivative(size_t index)
{
	// 	if (index < 0 || index >= m_derivativeFunctions.size())
	// 	{
	// 		std::cerr << "Derivative index out of range" << std::endl;
	// 		return glm::vec3(-15.15f);
	// 	}

	if (index == 0)
	{
		return DerivP1();
	}

	else if (index == 1)
	{
		return DerivP2();
	}

	std::cout << "CSF";
	//return m_derivativeFunctions[index](*this, m_points);
}

float Physics::StretchConstraint::ComputeScalingFactor()
{

	return ComputeConstraint() / (m_points[0]->m_invMass + m_points[1]->m_invMass);

}

void Physics::StretchConstraint::Solve(int iterationCount)
{
	// 	for (int i = 0; i < m_points.size(); ++i)
	// 	{
	// 		if (m_points[i]->m_isFixed)
	// 			continue;
	// 		glm::vec3 correction = -ComputeScalingFactor() * m_points[i]->m_invMass * ComputeDerivative(i);
	// 		m_points[i]->m_pos += correction * kp;
	// 	}

	float kp = 1 - std::powf(1 - m_stiffness, 1.f / iterationCount);
	float w1 = m_points[0]->m_invMass;
	float w2 = m_points[1]->m_invMass;

	float wSum = w1 + w2;
	if (wSum == 0)
	{
		return;
	}

	glm::vec3 diff = m_points[0]->m_pos - m_points[1]->m_pos;
	float len = glm::length(diff);

	glm::vec3 n = diff / len;

	glm::vec3 tempTerm = (len - m_l0) * n * kp;

	glm::vec3 correction1 = (-w1 / wSum) * tempTerm;
	glm::vec3 correction2 = (w2 / wSum) * tempTerm;

// 	float strain = m_l0 / 10.f;
// 	glm::vec3 min(-strain), max(strain);
// 	correction1 = glm::clamp(correction1, min, max);
// 	correction2 = glm::clamp(correction2, min, max);

	m_points[0]->m_pos += correction1;
	m_points[1]->m_pos += correction2;
}

