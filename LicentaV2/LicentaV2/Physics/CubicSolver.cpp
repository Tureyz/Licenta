#include "CubicSolver.h"

#include <iostream>
#include <algorithm>

#include "../Core/Utils.hpp"


float Physics::CubicSolver::computeTripleScalar(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float t)
{
	float valRet = glm::dot(glm::cross(x21 + t * v21, x31 + t * v31), x41 + t * v41);

	return valRet;
}

float Physics::CubicSolver::computeTripleScalarDeriv(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float t)
{
	glm::vec3 a = x21 + t * v21;
	glm::vec3 b = x31 + t * v31;
	glm::vec3 c = x41 + t * v41;

	return glm::dot(glm::cross(v21, b) + glm::cross(a, v31), c) + glm::dot(glm::cross(a, b), v41);
}

std::vector<float> Physics::CubicSolver::FindCriticalPoints(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41)
{

	const float eps = 1.f / 1000000;

	glm::vec3 v21v31 = glm::cross(v21, v31);
	glm::vec3 x21v31 = glm::cross(x21, v31);
	glm::vec3 v21x31 = glm::cross(v21, x31);
	glm::vec3 x21x31 = glm::cross(x21, x31);

	float v1v2v3 = glm::dot(v21v31, v41);
	float x1v2v3 = glm::dot(x21v31, v41);
	float v1x2v3 = glm::dot(v21x31, v41);
	float v1v2x3 = glm::dot(v21v31, x41);
	float x1x2v3 = glm::dot(x21x31, v41);
	float x1v2x3 = glm::dot(x21v31, x41);
	float v1x2x3 = glm::dot(v21x31, x41);
	float x1x2x3 = glm::dot(x21x31, x41);

	float a = v1v2v3;
	float b = x1v2v3 + v1x2v3 + v1v2x3;
	float c = x1x2v3 + x1v2x3 + v1x2x3;
	float d = x1x2x3;

	float aDeriv = 3 * a;
	float bDeriv = 2 * b;
	float cDeriv = c;

	float deltaDeriv = 4 * (b * b - aDeriv * c);

	float sqr = std::sqrt(deltaDeriv);
	float doubleA = 2 * aDeriv;

	float t1Deriv = ((-bDeriv + sqr) / doubleA);
	float t2Deriv = ((-bDeriv - sqr) / doubleA);

	std::vector<float> result;

	if (deltaDeriv >= eps)
	{
		if (t1Deriv >= 0 && t1Deriv <= 1)
		{
			result.push_back(t1Deriv);
		}
		if (t2Deriv >= 0 && t2Deriv <= 1)
		{
			result.push_back(t2Deriv);
		}
	}
	else if (std::abs(deltaDeriv) < eps)
	{
		if (t1Deriv >= 0 && t1Deriv <= 1)
		{
			result.push_back(t1Deriv);
		}
	}

	return result;
}

float Physics::CubicSolver::Secant(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float &a, const float &b, const int &maxIterations, const float &tolerance)
{
	float x0 = a, x1 = b;

	for (int i = 0; i < maxIterations; ++i)
	{
		float fx0 = computeTripleScalar(x21, x31, x41, v21, v31, v41, x0);
		float fx1 = computeTripleScalar(x21, x31, x41, v21, v31, v41, x1);

		float xcrt = (x0 * fx1 - x1 * fx0) / (fx1 - fx0);

		if (std::abs(xcrt - x1) < tolerance)
		{
			//std::cout << "Found solution " << xcrt << " after " << i << " iterations." << std::endl;
			return xcrt;
		}

		x0 = x1;
		x1 = xcrt;
	}

	return -15;
}

std::vector<float> Physics::CubicSolver::NewtonRaphson(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float &initialGuess, const int &maxIterations, const float &tolerance)
{

	std::vector<float> result;
	float xprev = initialGuess;

	for (int i = 0; i < maxIterations; ++i)
	{
		float fCrt = computeTripleScalar(x21, x31, x41, v21, v31, v41, xprev);
		float fCrtPrime = computeTripleScalarDeriv(x21, x31, x41, v21, v31, v41, xprev);
		if (fCrtPrime == 0)
			fCrtPrime += 2 * tolerance;

		float xcrt = xprev - fCrt / fCrtPrime;

		if (std::abs(xcrt - xprev) < tolerance && xcrt >= intervalMin && xcrt <= intervalMax)
		{

			result.push_back(xcrt);
			//std::cout << "Found solution " << xcrt << " after " << i << " iterations." << std::endl;
			return result;
		}

		xprev = xcrt;
	}

	return result;

}

std::vector<float> Physics::CubicSolver::FindCoplanarityTimes(const glm::vec3 &x1, const glm::vec3 &x2, const glm::vec3 &x3, const glm::vec3 &x4, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3, const glm::vec3 &v4)
{
	std::vector<float> result;


	glm::vec3 x21 = x2 - x1;
	glm::vec3 x31 = x3 - x1;
	glm::vec3 x41 = x4 - x1;

	glm::vec3 v21 = v2 - v1;
	glm::vec3 v31 = v3 - v1;
	glm::vec3 v41 = v4 - v1;


	std::vector<float> criticalPoints = Physics::CubicSolver::FindCriticalPoints(x21, x31, x41, v21, v31, v41);


// 	if (criticalPoints.size() > 1)
// 	{
// 		std::cout << "Critical points: ";
// 
// 		for (auto p : criticalPoints)
// 		{
// 			std::cout << p << ", ";
// 		}
// 
// 		std::cout << std::endl;
// 	}

	std::vector<float> intervalEndpoints({ intervalMin, intervalMax });
	intervalEndpoints.insert(intervalEndpoints.end(), criticalPoints.begin(), criticalPoints.end());
	std::sort(intervalEndpoints.begin(), intervalEndpoints.end());


	std::vector<Interval> intervals;
	const float eps = 1.f / 1000000;


	for (int i = 0; i < intervalEndpoints.size() - 1; ++i)
	{
		float a = intervalEndpoints[i];
		float b = intervalEndpoints[i + 1];
		float fa = computeTripleScalar(x21, x31, x41, v21, v31, v41, a);
		float fb = computeTripleScalar(x21, x31, x41, v21, v31, v41, b);

		if (std::abs(fa) < eps)
		{
			result.push_back(a);
		}

		if (fa * fb < 0)
		{
			float t = Secant(x21, x31, x41, v21, v31, v41, a, b, 20, eps);

			if (t != -15)
			{
				result.push_back(t);
			}
		}
	}


	std::sort(result.begin(), result.end());
	return result;
	//return NewtonRaphson(x21, x31, x41, v21, v31, v41, 0, 20, 1.f / 1000000);
	//return result;
}


// std::vector<float> Collision::NarrowSpatialHashing::FindCoplanarity(const DeformingPoint &p, const DeformingTriangle &t) const
// {
// 	glm::vec3 x1 = p.m_node->m_pos;
// 	glm::vec3 x2 = t.m_triangle->m_nodes[0]->m_pos;
// 	glm::vec3 x3 = t.m_triangle->m_nodes[1]->m_pos;
// 	glm::vec3 x4 = t.m_triangle->m_nodes[2]->m_pos;
// 
// 	glm::vec3 v1 = p.m_node->m_projection - x1;
// 	glm::vec3 v2 = t.m_triangle->m_nodes[0]->m_projection - x2;
// 	glm::vec3 v3 = t.m_triangle->m_nodes[1]->m_projection - x3;
// 	glm::vec3 v4 = t.m_triangle->m_nodes[2]->m_projection - x4;
// 
// 	glm::vec3 x21 = x2 - x1;
// 	glm::vec3 x31 = x3 - x1;
// 	glm::vec3 x41 = x4 - x1;
// 
// 	glm::vec3 v21 = v2 - v1;
// 	glm::vec3 v31 = v3 - v1;
// 	glm::vec3 v41 = v4 - v1;
// 
// 	return NewtonRaphson(x21, x31, x41, v21, v31, v41, Core::TIME_STEP / 2, 20, 1.f / 100000);
// }
// 
// std::vector<float> Collision::NarrowSpatialHashing::FindCoplanarity(const DeformingEdge &e1, const DeformingEdge &e2) const
// {
// 	glm::vec3 x1 = e1.m_edge->m_v1->m_pos;
// 	glm::vec3 x2 = e1.m_edge->m_v2->m_pos;
// 	glm::vec3 x3 = e2.m_edge->m_v1->m_pos;
// 	glm::vec3 x4 = e2.m_edge->m_v2->m_pos;
// 
// 	glm::vec3 v1 = e1.m_edge->m_v1->m_projection - x1;
// 	glm::vec3 v2 = e1.m_edge->m_v2->m_projection - x2;
// 	glm::vec3 v3 = e2.m_edge->m_v1->m_projection - x3;
// 	glm::vec3 v4 = e2.m_edge->m_v2->m_projection - x4;
// 
// 	glm::vec3 x21 = x2 - x1;
// 	glm::vec3 x31 = x3 - x1;
// 	glm::vec3 x41 = x4 - x1;
// 
// 	glm::vec3 v21 = v2 - v1;
// 	glm::vec3 v31 = v3 - v1;
// 	glm::vec3 v41 = v4 - v1;
// 
// 	return NewtonRaphson(x21, x31, x41, v21, v31, v41, Core::TIME_STEP / 2, 20, 1.f / 100000);
// }
