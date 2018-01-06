#pragma once
#include<vector>

#include "../Dependencies/glm/glm.hpp"
#include "../Core/Utils.hpp"

namespace Physics
{
	

	static const float intervalMin = 0;
	static const float intervalMax = Core::TIME_STEP;

	class CubicSolver
	{
	public:

// 		static std::vector<float> FindCoplanarity(const DeformingPoint &p, const DeformingTriangle &t) const;
// 		static std::vector<float> FindCoplanarity(const DeformingEdge &e1, const DeformingEdge &e2) const;

		static std::vector<float> FindCoplanarityTimes(const glm::vec3 &x1, const glm::vec3 &x2, const glm::vec3 &x3, const glm::vec3 &x4, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &v3, const glm::vec3 &v4);

	private:

		class Interval
		{
		public:
			Interval(float a, float b, float mid, float midValue) : m_a(a), m_b(b), m_mid(mid), m_midValue(midValue) {}
			float m_a, m_b, m_mid, m_midValue;
		};

		static float Secant(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float &a, const float &b, const int &maxIterations, const float &tolerance);
		static float computeTripleScalar(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float t);
		static float computeTripleScalarDeriv(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float t);
		static std::vector<float> FindCriticalPoints(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41);
		static std::vector<float> NewtonRaphson(const glm::vec3 &x21, const glm::vec3 &x31, const glm::vec3 &x41, const glm::vec3 &v21, const glm::vec3 &v31, const glm::vec3 &v41, const float &initialGuess, const int &maxIterations, const float &tolerance);


	};
}