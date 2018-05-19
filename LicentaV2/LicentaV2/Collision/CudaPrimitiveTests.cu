#include "CudaPrimitiveTests.cuh"

#include "../Core/CudaUtils.cuh"

void CudaPrimitiveTests::DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, const thrust::device_vector<float3> &positions,
	thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, const float thickness)
{

	const int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	DCDTriangleTests << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(thrust::raw_pointer_cast(&triangles[0]), thrust::raw_pointer_cast(&positions[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize, thickness);

}

__device__ bool CudaPrimitiveTests::VFSanity(const int p, const int v1, const int v2, const int v3)
{
	return !(p == v1 || p == v2 || p == v3);
}

__device__ bool CudaPrimitiveTests::EESanity(const int p1, const int p2, const int q1, const int q2)
{
	return !(p1 == q1 || p1 == q2 || p2 == q1 || p2 == q2);
}

__global__ void CudaPrimitiveTests::DCDTriangleTests(const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
	Physics::PrimitiveContact * __restrict__ vfContacts, const uint64_t vfSize, Physics::PrimitiveContact * __restrict__ eeContacts, const uint64_t eeSize, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= vfSize + eeSize)
		return;

	if (id < vfSize)
	{
		DCDTestVFs(id, triangles, positions, vfContacts, thickness);
	}
	else
	{
		DCDTestEEs(id - vfSize, triangles, positions, eeContacts, thickness);
	}
}

__device__ void CudaPrimitiveTests::DCDTestVFs(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
	Physics::PrimitiveContact * __restrict__ vfContacts, const float thickness)
{


	Physics::PrimitiveContact myContact = vfContacts[id];

	if (!VFSanity(myContact.v1, myContact.v2, myContact.v3, myContact.v4))
		return;

	float3 x4 = positions[myContact.v1];
	float3 x1 = positions[myContact.v2];
	float3 x2 = positions[myContact.v3];
	float3 x3 = positions[myContact.v4];

	//printf("[%d] v1: %d, v2: %d, v3: %d, v4: %d\n", id, myContact.v1, myContact.v2, myContact.v3, myContact.v4);

	float3 x13 = x1 - x3;
	float3 x23 = x2 - x3;
	float3 x43 = x4 - x3;

	float3 crs = CudaUtils::cross(x2 - x1, x3 - x1);

	float3 n = CudaUtils::normalize(crs);


	if (fabsf(CudaUtils::dot(x43, n)) < thickness)
	{
		float dTemp = CudaUtils::dot(x13, x23);
		float m11 = CudaUtils::dot(x13, x13);
		float m12 = dTemp;
		float m21 = dTemp;
		float m22 = CudaUtils::dot(x23, x23);

		float vr1 = CudaUtils::dot(x13, x43);
		float vr2 = CudaUtils::dot(x23, x43);

		float w2 = (m11 * vr2 - m21 * vr1) / (m11 * m22 + m21 * m12);
		float w1 = (vr1 - m12 * w2) / m11;
		float w3 = 1.f - w1 - w2;

		if (w1 < 0 || w2 < 0 || w3 < 0)
			return;

		float triangleArea = CudaUtils::len(crs) / 2.f;
		float delta = thickness / sqrtf(triangleArea);

		//printf("[%d] bp\n", id);
		if ((w1 >= -delta && w1 <= 1 + delta) && (w2 >= -delta && w2 <= 1 + delta) && (w3 >= -delta && w3 <= 1 + delta))
		{

			myContact.w1 = 1.f;
			myContact.w2 = w1;
			myContact.w3 = w2;
			myContact.w4 = w3;
			myContact.n = n;
			myContact.t = 0.f;
			myContact.contact = true;

			vfContacts[id] = myContact;

			if (CudaUtils::isNan(n))
			{
				printf("[%d] VF n nan\n", id);
			}
		}
	}
}

__device__ void CudaPrimitiveTests::DCDTestEEs(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
	Physics::PrimitiveContact * __restrict__ eeContacts, const float thickness)
{
	Physics::PrimitiveContact myContact = eeContacts[id];

	if (!EESanity(myContact.v1, myContact.v2, myContact.v3, myContact.v4))
		return;

	//printf("[%d] v1: %d, v2: %d, v3: %d, v4: %d\n", id, myContact.v1, myContact.v2, myContact.v3, myContact.v4);

	float3 p1 = positions[myContact.v1];
	float3 p2 = positions[myContact.v2];
	float3 q1 = positions[myContact.v3];
	float3 q2 = positions[myContact.v4];

	float3 a = p2 - p1;
	float3 b = q2 - q1;
	float3 c = p1 - q1;


	float bc = CudaUtils::dot(b, c);
	float aa = CudaUtils::dot(a, a);
	float ab = CudaUtils::dot(a, b);
	float ac = CudaUtils::dot(a, c);
	float bb = CudaUtils::dot(b, b);

	//printf("[%d] bc: %f, aa: %f, ab: %f, ac: %f, bb: %f\n", id, bc, aa, ab, ac, bb);

	float t = (-bc * aa) / (ab * ab - ab * ac - aa * bb);
	float s = (t * ab - ac) / aa;


	t = CudaUtils::clamp(t, 0.f, 1.f);
	s = CudaUtils::clamp(s, 0.f, 1.f);

	//printf("[%d] s: %f, t: %f\n", id, s, t);
	float3 cp1 = (p1 + s * a);
	float3 cp2 = (q1 + t * b);

	if (CudaUtils::distance(cp1, cp2) < thickness)
	{
		myContact.w1 = 1.f - t;
		myContact.w2 = t;
		myContact.w3 = 1.f - s;
		myContact.w4 = s;
		//printf("[%d] bp\n", id);
		myContact.n = CudaUtils::normalize(cp2 - cp1);
		myContact.t = 0.f;
		myContact.contact = true;

		eeContacts[id] = myContact;
	}

}