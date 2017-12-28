#include "BendConstraint.h"
#include <iostream>
#include "../Core/Utils.hpp"

Physics::BendConstraint::BendConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type)
{
	m_points = points;
	m_stiffness = stiffness;
	m_type = type;
}

void Physics::BendConstraint::Init()
{
	glm::vec3 p1 = m_points[0]->m_pos, p2 = m_points[1]->m_pos, p3 = m_points[2]->m_pos, p4 = m_points[3]->m_pos;

	glm::vec3 p12 = p2 - p1;



	m_initialAngle = std::acos(glm::dot(glm::normalize(glm::cross(p12, p3 - p1)), glm::normalize(glm::cross(p12, p4 - p1))));
}

float Physics::BendConstraint::ComputeConstraint()
{
	return -1;
}

glm::vec3 Physics::BendConstraint::ComputeDerivative(size_t index)
{
	return glm::vec3(15);
}

float Physics::BendConstraint::ComputeScalingFactor()
{
	return 15;
}

void Physics::BendConstraint::Solve(int iterationCount)
{
	glm::vec3 p1 = m_points[0]->m_pos, p2 = m_points[1]->m_pos, p3 = m_points[2]->m_pos, p4 = m_points[3]->m_pos;
	float w1 = m_points[0]->m_invMass, w2 = m_points[1]->m_invMass, w3 = m_points[2]->m_invMass, w4 = m_points[3]->m_invMass;

// 	if (w1 == 0 || w2 == 0 || w3 == 0 || w4 == 0)
// 	{
// 		std::cout << "aici\n";
// 	}
	
	glm::vec3 p12 = p2 - p1;
	glm::vec3 c23 = glm::cross(p2, p3), c24 = glm::cross(p2, p4);
	glm::vec3 n1 = glm::length(c23) == 0 ? glm::vec3(0) : glm::normalize(c23);
	glm::vec3 n2 = glm::length(c24) == 0 ? glm::vec3(0) : glm::normalize(c24);

	float d = Core::Utils::clamp(glm::dot(n1, n2), -1, 1);


	glm::vec3 q3 = (glm::cross(p2, n2) + (glm::cross(n1, p2) * d)) / glm::length(glm::cross(p2, p3));
	glm::vec3 q4 = (glm::cross(p2, n1) + (glm::cross(n2, p2) * d)) / glm::length(glm::cross(p2, p4));
	glm::vec3 q2 = -((glm::cross(p3, n2) + (glm::cross(n1, p3) * d)) / glm::length(glm::cross(p2, p3))) - ((glm::cross(p4, n1) + (glm::cross(n2, p4) * d)) / glm::length(glm::cross(p2, p4)));
	glm::vec3 q1 = -q2 - q3 - q4;

	float kp = 1 - std::powf(1 - m_stiffness, 1.f / iterationCount);
	float term = std::sqrt(1 - d * d) * (std::acos(d) - m_initialAngle) * kp;

	float l1 = glm::length(q1), l2 = glm::length(q2), l3 = glm::length(q3), l4 = glm::length(q4);

	float scalingFactor = w1 * l1 * l1 + w2 * l2 * l2 + w3 * l3 * l3 + w4 * l4 * l4;

	if (scalingFactor == 0)
	{
		return;
	}
	glm::vec3 correction1 = -(w1 * term / scalingFactor) * q1;
	glm::vec3 correction2 = -(w2 * term / scalingFactor) * q2;
	glm::vec3 correction3 = -(w3 * term / scalingFactor) * q3;
	glm::vec3 correction4 = -(w4 * term / scalingFactor) * q4;

// 	auto asd = glm::isnan(correction1);
// 	if (asd.x == true || asd.y == true || asd.z == true)
// 	{
// 		std::cout << "woops\n";
// 	}
// 
// 	asd = glm::isnan(correction2);
// 	if (asd.x == true || asd.y == true || asd.z == true)
// 	{
// 		std::cout << "woops2\n";
// 	}
// 
// 	asd = glm::isnan(correction3);
// 	if (asd.x == true || asd.y == true || asd.z == true)
// 	{
// 		std::cout << "woops3\n";
// 	}
// 
// 	asd = glm::isnan(correction4);
// 	if (asd.x == true || asd.y == true || asd.z == true)
// 	{
// 		std::cout << "woops4\n";
// 	}
	m_points[0]->m_pos += correction1;
	m_points[1]->m_pos += correction2;
	m_points[2]->m_pos += correction3;
	m_points[3]->m_pos += correction4;
}

