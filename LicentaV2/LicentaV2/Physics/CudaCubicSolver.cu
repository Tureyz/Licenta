#include "CudaCubicSolver.cuh"

#include "../Core/CudaUtils.cuh"

__device__ void Physics::FindCoplanarityTimes(const float3 & x1, const float3 & x2, const float3 & x3, const float3 & x4,
	const float3 & v1, const float3 & v2, const float3 & v3, const float3 & v4, float * times, int & timesCount)
{
	timesCount = 0;

	const float3 x21 = x2 - x1;
	const float3 x31 = x3 - x1;
	const float3 x41 = x4 - x1;

	const float3 v21 = v2 - v1;
	const float3 v31 = v3 - v1;
	const float3 v41 = v4 - v1;


	float critPoint1, critPoint2;
	int critPointCount = 0;

	FindCriticalPoints(x21, x31, x41, v21, v31, v41, critPoint1, critPoint2, critPointCount);


	float intervalEndPoints[4];

	int intervalCount = 0;
	intervalEndPoints[intervalCount++] = 0.f;

	
	if (critPointCount == 1)
	{
		intervalEndPoints[intervalCount++] = critPoint1;
	}
	else if (critPointCount == 2)
	{

		intervalEndPoints[intervalCount++] = critPoint1;
		intervalEndPoints[intervalCount++] = critPoint2;
	}

	intervalEndPoints[intervalCount++] = 1.f;

	for (int i = 0; i < intervalCount - 1; ++i)
	{
		const float a = intervalEndPoints[i];
		const float b = intervalEndPoints[i + 1];

		const float fa = ComputeTripleScalar(x21, x31, x41, v21, v31, v41, a);
		const float fb = ComputeTripleScalar(x21, x31, x41, v21, v31, v41, b);

		if (fabsf(fa) < EPS)
		{
			times[timesCount++] = a;
		}

		if (fa * fb < 0.f)
		{
			float t = Secant(x21, x31, x41, v21, v31, v41, a, b, 20);
			if (t != -15)
			{
				times[timesCount++] = t;
			}
		}

	}
}

__device__ void Physics::FindCriticalPoints(const float3 & x21, const float3 & x31, const float3 & x41,
	const float3 & v21, const float3 & v31, const float3 & v41, float & p1, float & p2, int & pointsCount)
{
	const float3 v21v31 = CudaUtils::cross(v21, v31);
	const float3 x21v31 = CudaUtils::cross(x21, v31);
	const float3 v21x31 = CudaUtils::cross(v21, x31);
	const float3 x21x31 = CudaUtils::cross(x21, x31);

	const float v1v2v3 = CudaUtils::dot(v21v31, v41);
	const float x1v2v3 = CudaUtils::dot(x21v31, v41);
	const float v1x2v3 = CudaUtils::dot(v21x31, v41);
	const float v1v2x3 = CudaUtils::dot(v21v31, x41);
	const float x1x2v3 = CudaUtils::dot(x21x31, v41);
	const float x1v2x3 = CudaUtils::dot(x21v31, x41);
	const float v1x2x3 = CudaUtils::dot(v21x31, x41);
	const float x1x2x3 = CudaUtils::dot(x21x31, x41);

	const float a = v1v2v3;
	const float b = x1v2v3 + v1x2v3 + v1v2x3;
	const float c = x1x2v3 + x1v2x3 + v1x2x3;
	//const float d = x1x2x3;

	const float aDeriv = 3 * a;
	const float bDeriv = 2 * b;
	//const float cDeriv = c;

	const float deltaDeriv = 4 * (b * b - aDeriv * c);

	const float sqr = sqrtf(deltaDeriv);
	const float doubleA = 2 * aDeriv;

	const float t1Deriv = ((-bDeriv + sqr) / doubleA);
	const float t2Deriv = ((-bDeriv - sqr) / doubleA);	

	if (deltaDeriv >= EPS)
	{
		if (CudaUtils::isBetween(t1Deriv, 0.f, 1.f))
		{
			p1 = t1Deriv;
			pointsCount++;
		}
		if (CudaUtils::isBetween(t2Deriv, 0.f, 1.f))
		{
			p2 = t2Deriv;
			pointsCount++;
		}
	}
	else if (fabsf(deltaDeriv) < EPS)
	{
		if (CudaUtils::isBetween(t1Deriv, 0.f, 1.f))
		{
			p1 = t1Deriv;
			pointsCount++;
		}
	}
}

__device__ float Physics::ComputeTripleScalar(const float3 & x21, const float3 & x31, const float3 & x41, const float3 & v21, const float3 & v31, const float3 & v41, const float t)
{
	return CudaUtils::dot(CudaUtils::cross(x21 + t * v21, x31 + t * v31), x41 + t * v41);
}

__device__ float Physics::Secant(const float3 & x21, const float3 & x31, const float3 & x41, const float3 & v21, const float3 & v31, const float3 & v41,
	const float a, const float b, const int maxIterations)
{
	float x0 = a, x1 = b;

	for (int i = 0; i < maxIterations; ++i)
	{
		float fx0 = ComputeTripleScalar(x21, x31, x41, v21, v31, v41, x0);
		float fx1 = ComputeTripleScalar(x21, x31, x41, v21, v31, v41, x1);

		float xcrt = (x0 * fx1 - x1 * fx0) / (fx1 - fx0);

		if (fabsf(xcrt - x1) < EPS)
		{
			return xcrt;
		}

		x0 = x1;
		x1 = xcrt;
	}

	return -15.f;
}


