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
		DCDTestEEsBridson(id - vfSize, triangles, positions, eeContacts, thickness, 0.000000001f);
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
		//x43 = CudaUtils::ProjectOnPlane(x4, x3, n) - x3;

		float dTemp = CudaUtils::dot(x13, x23);
		float m11 = CudaUtils::dot(x13, x13);
		float m12 = dTemp;
		float m21 = dTemp;
		float m22 = CudaUtils::dot(x23, x23);

		float vr1 = CudaUtils::dot(x13, x43);
		float vr2 = CudaUtils::dot(x23, x43);

		float w2 = (m11 * vr2 - m21 * vr1) / (m11 * m22 - m21 * m12);
		float w1 = (vr1 - m12 * w2) / m11;
		float w3 = 1.f - w1 - w2;

		float triangleArea = CudaUtils::len(crs) / 2.f;
		float delta = thickness / sqrtf(triangleArea);

		delta = thickness / (CudaUtils::len(x13) + CudaUtils::len(x23) + CudaUtils::len(x1 - x2));

		//printf("[%d] thickness: %f, delta: %f\n", id, thickness, delta);
		//const float min = -delta;
		const float min = 0.f;
		const float max = 1 + delta;

		if (CudaUtils::isBetween(w1, min, max) && CudaUtils::isBetween(w2, min, max) && CudaUtils::isBetween(w3, min, max))
		{

			myContact.w1 = 1.f;
			myContact.w2 = -w1;
			myContact.w3 = -w2;
			myContact.w4 = -w3;
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

__device__ void CudaPrimitiveTests::DCDTestVFs2(const int id, const Physics::CudaTriangle *__restrict__ triangles, const float3 *__restrict__ positions, Physics::PrimitiveContact *__restrict__ vfContacts, const float thickness)
{
	Physics::PrimitiveContact myContact = vfContacts[id];

	if (!VFSanity(myContact.v1, myContact.v2, myContact.v3, myContact.v4))
		return;

	float3 x4 = positions[myContact.v1];
	float3 x1 = positions[myContact.v2];
	float3 x2 = positions[myContact.v3];
	float3 x3 = positions[myContact.v4];


	float3 u = x2 - x1;
	float3 v = x3 - x1;
	float3 w = x4 - x1;

	float3 crs = CudaUtils::cross(u, v);

	float3 n = CudaUtils::normalize(crs);


	if (fabsf(CudaUtils::dot(w, n)) < thickness)
	{

		w = CudaUtils::ProjectOnPlane(x4, x1, n) - x1;

		float triangleArea = CudaUtils::len(crs) / 2.f;
		float delta = thickness / sqrtf(triangleArea);

		printf("[%d] delta: %f\n", id, delta);
		const float min = -delta;
		const float max = 1 + delta;

		float3 vxw = CudaUtils::cross(v, w);
		float3 vxu = CudaUtils::cross(v, u);


		if (CudaUtils::dot(vxw, vxu) < min)
			return;

		float3 uxw = CudaUtils::cross(u, w);
		float3 uxv = CudaUtils::cross(u, v);

		if (CudaUtils::dot(uxw, uxv) < min)
			return;

		float denom = CudaUtils::len(uxv);

		float r = CudaUtils::len(vxw) / denom;
		float t = CudaUtils::len(uxw) / denom;
		float s = 1 - r - t;

		if (r <= max && t <= max && s <= max)
		{
			myContact.w1 = 1.f;
			myContact.w2 = s;
			myContact.w3 = r;
			myContact.w4 = t;
			myContact.n = n;
			myContact.t = 0.f;
			myContact.contact = true;

			vfContacts[id] = myContact;
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
		myContact.w1 = (1.f - s);
		myContact.w2 = s;
		myContact.w3 = -(1.f - t);
		myContact.w4 = -t;
		//printf("[%d] bp\n", id);
		myContact.n = CudaUtils::normalize(cp1 - cp2);
		myContact.t = 0.f;
		myContact.contact = true;

		eeContacts[id] = myContact;
	}

}

__device__ void CudaPrimitiveTests::DCDTestEEsBridson(const int id, const Physics::CudaTriangle *__restrict__ triangles,
	const float3 *__restrict__ positions, Physics::PrimitiveContact *__restrict__ eeContacts, const float thickness, const float eps)
{
	Physics::PrimitiveContact myContact = eeContacts[id];

	if (!EESanity(myContact.v1, myContact.v2, myContact.v3, myContact.v4))
		return;

	//printf("[%d] v1: %d, v2: %d, v3: %d, v4: %d\n", id, myContact.v1, myContact.v2, myContact.v3, myContact.v4);

	float3 x1 = positions[myContact.v1];
	float3 x2 = positions[myContact.v2];
	float3 x3 = positions[myContact.v3];
	float3 x4 = positions[myContact.v4];

	float3 x21 = x2 - x1;
	float3 x31 = x3 - x1;
	float3 x43 = x4 - x3;

	float3 crs = CudaUtils::cross(x21, x43);


	if (CudaUtils::len(crs) < eps)
	{
		if (TestEdgeDegenerate(positions, myContact, thickness))
		{
			eeContacts[id] = myContact;
		}

		return;
	}

	if (CudaUtils::isZero(crs))
	{
		printf("[%d] crs zero, x1: (%f, %f, %f), x2: (%f, %f, %f), x3: (%f, %f, %f), x4: (%f, %f, %f) \n", id,
			x1.x, x1.y, x1.z, x2.x, x2.y, x2.z, x3.x, x3.y, x3.z, x4.x, x4.y, x4.z);

		return;
	}


	float dTemp = CudaUtils::dot(-x21, x43);

	float m11 = CudaUtils::dot(x21, x21);
	float m12 = dTemp;
	float m21 = dTemp;
	float m22 = CudaUtils::dot(x43, x43);

	float vr1 = CudaUtils::dot(x21, x31);
	float vr2 = CudaUtils::dot(-x43, x31);

	float b = (m11 * vr2 - m21 * vr1) / (m11 * m22 - m21 * m12);
	float a = (vr1 - m12 * b) / m11;

	if (CudaUtils::isBetween(a, 0.f, 1.f) && CudaUtils::isBetween(b, 0.f, 1.f))
	{
		float3 cp1 = x1 = a * x21;
		float3 cp2 = x3 = a * x43;

		if (CudaUtils::distance(cp1, cp2) < thickness)
		{
			myContact.w1 = 1 - a;
			myContact.w2 = a;
			myContact.w3 = -(1 - b);
			myContact.w4 = -b;
			myContact.contact = true;

			float3 dir = (x1 + a * x21) - (x3 + b * x43);

			myContact.n = CudaUtils::normalize(CudaUtils::isZero(dir) ? crs : dir);

			eeContacts[id] = myContact;
			return;
		}
	}
	else
	{
		float aClamped = CudaUtils::clamp(a, 0.f, 1.f);
		float bClamped = CudaUtils::clamp(b, 0.f, 1.f);

		float d1 = fabsf(a - aClamped) * CudaUtils::len(x21);
		float d2 = fabsf(b - bClamped) * CudaUtils::len(x43);

		a = aClamped;
		b = bClamped;

		float3 cp1, cp2;

		if (d1 > d2)
		{
			cp1 = x1 + a * x21;
			float3 p2Raw = CudaUtils::ProjectOnLine(cp1, x3, x4);
			cp2 = CudaUtils::ClampOnLine(p2Raw, x3, x4);
		}
		else
		{
			cp2 = x3 + b * x43;
			float3 p1Raw = CudaUtils::ProjectOnLine(cp2, x1, x2);
			cp1 = CudaUtils::ClampOnLine(p1Raw, x1, x2);
		}

		if (CudaUtils::distance(cp1, cp2) < thickness)
		{
			myContact.w1 = 1 - a;
			myContact.w2 = a;
			myContact.w3 = -(1 - b);
			myContact.w4 = -b;
			myContact.n = CudaUtils::normalize(cp1 - cp2);
			myContact.t = 0.f;
			myContact.contact = true;
			eeContacts[id] = myContact;
		}
	}
}

__device__ bool CudaPrimitiveTests::TestEdgeDegenerate(const float3 * __restrict__ positions, Physics::PrimitiveContact &contact, const float thickness)
{
	float3 x21 = positions[contact.v2] - positions[contact.v1];
	float3 x43 = positions[contact.v4] - positions[contact.v3];

	float3 mid1 = positions[contact.v1] + x21 / 2.f;
	float3 mid2 = positions[contact.v3] + x43 / 2.f;

	if (CudaUtils::distance(mid1, mid2) < thickness)
	{
		contact.w1 = 0.5f;
		contact.w2 = 0.5f;
		contact.w3 = 0.5f;
		contact.w4 = 0.5f;
		contact.n = CudaUtils::normalize(mid1 - mid2);
		contact.contact = true;

		return true;
	}

	return false;
}
