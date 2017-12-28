#include "Constraint.h"

#include <iostream>

// void Physics::Constraint::FixedUpdate()
// {
// // 	for (int i = 0; i < m_derivativeFunctions.size(); ++i)
// // 	{
// // 		m_derivativeValues[i] = ComputeDerivative(i);
// // 	}
// 
// 	ComputeDerivatives();
// 	m_scalingFactor = ComputeScalingFactor();
// 	
// }

// float Physics::Constraint::StretchFunc(glm::vec3 p1, glm::vec3 p2, float l0)
// {
// 	return glm::length(p1 - p2) - l0;	
// }
// 
// float Physics::Constraint::BendingFunc(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, float fi0)
// {
// 	glm::vec3 term1 = glm::cross(p2 - p1, p3 - p1);
// 	glm::vec3 term2 = glm::cross(p2 - p1, p4 - p1);
// 
// 	return acos(glm::dot(term1 / glm::length(term1), term2 / glm::length(term2))) - fi0;
// }
