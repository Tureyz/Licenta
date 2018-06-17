#include "CudaUtils.cuh"


__device__ static const float EPS = 0.00000001f;

void CudaUtils::CudaTimer::Start()
{
	cudaEventRecord(m_start, 0);
}

float CudaUtils::CudaTimer::End()
{
	cudaEventRecord(m_end, 0);
	cudaEventSynchronize(m_end);
	float elapsed;
	cudaEventElapsedTime(&elapsed, m_start, m_end);
	return elapsed;
}

CudaUtils::CudaTimer::CudaTimer()
{
	cudaEventCreate(&m_start);
	cudaEventCreate(&m_end);
}

CudaUtils::CudaTimer::~CudaTimer()
{
	cudaEventDestroy(m_start);
	cudaEventDestroy(m_end);
}

__host__ __device__ float3 operator*(const float3 & a, const float3 & b)
{
	return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

__host__ __device__ float3 operator*(const float s, const float3 &a)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

__host__ __device__ float3 operator*(const float3 &a, const float s)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

__host__ __device__ float3 operator/(const float3 &a, const float b)
{
	return make_float3(a.x / b, a.y / b, a.z / b);
}

__host__ __device__ float3 operator-(const float3 &a, const float3 &b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__host__ __device__ float3 operator-(const float3 &a, const float b)
{
	return make_float3(a.x - b, a.y - b, a.z - b);
}

__host__ __device__ float3 operator+(const float3 &a, const float b)
{
	return make_float3(a.x + b, a.y + b, a.z + b);
}

__host__ __device__ void operator*=(float3 &a, const float s)
{
	a.x *= s; a.y *= s; a.z *= s;
}

__host__ __device__ void operator/=(float3 & a, const float s)
{
	assert(s != 0.0f);

	a.x /= s; a.y /= s; a.z /= s;
}

__host__ __device__ float3 operator+(const float3 &a, const float3 &b)
{
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__host__ __device__ float3 operator/(const float3 &a, const float3 &b)
{
	return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}

__host__ __device__ void operator+=(float3 & a, const float3 & b)
{
	a.x += b.x; a.y += b.y; a.z += b.z;
}

__host__ __device__ bool operator==(const float3 &a, const float3 &b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

__host__ __device__ bool operator!=(const float3 &a, const float3 &b)
{
	return !(a == b);
}

__host__ __device__ float3 operator-(const float3 & a)
{
	return make_float3(-a.x, -a.y, -a.z);
}

std::string CudaUtils::MemUsage(const double freeInit)
{
	uint64_t freeVRAM, totalVRAM;

	cudaMemGetInfo(&freeVRAM, &totalVRAM);
	
	double free = (double) freeVRAM, total = (double) totalVRAM, used = total - free;

	if (freeInit == -1)
		return "GPU memory usage: " + std::to_string(free / 1024.0 / 1024.0) + " MB free / " + std::to_string(total / 1024.0 / 1024.0) + " MB total";
	
	return "GPU memory usage: " + std::to_string((freeInit - free) / 1024.0 / 1024.0) + " MB used / " + std::to_string(freeInit / 1024.0 / 1024.0) + " MB total";
}

double CudaUtils::VRAMUsage()
{
	uint64_t freeVRAM, totalVRAM;

	cudaMemGetInfo(&freeVRAM, &totalVRAM);

	return (double)freeVRAM;
}



glm::vec3 CudaUtils::MakeVec(const float3 &a)
{
	return glm::vec3(a.x, a.y, a.z);
}

__host__ __device__ bool CudaUtils::isZero(const float3 & vec)
{
	return vec.x == 0.f && vec.y == 0.f && vec.z == 0.f;
}

__device__ float3 CudaUtils::ProjectOnPlane(const float3 & point, const float3 & planePoint, const float3 & planeNormal)
{
	return point + ((dot(planePoint, planeNormal) - dot(point, planeNormal)) / dot(planeNormal, planeNormal)) * planeNormal;
}

__device__ float3 CudaUtils::ProjectOnLine(const float3 & point, const float3 & edgeP1, const float3 & edgeP2)
{
	float3 ab = edgeP2 - edgeP1;
	float3 ap = point - edgeP1;

	return edgeP1 + CudaUtils::dot(ap, ab) / CudaUtils::dot(ab, ab) * ab;
}

__device__ float3 CudaUtils::ClampOnLine(const float3 & point, const float3 & edgeP1, const float3 & edgeP2)
{
	float distP1 = CudaUtils::distance(point, edgeP1);
	float distP2 = CudaUtils::distance(point, edgeP2);

	if (fabsf(distP1 + distP2 - CudaUtils::distance(edgeP1, edgeP2)) < EPS)
	{
		return point;
	}

	return distP1 < distP2 ? edgeP1 : edgeP2;
}

__device__ bool CudaUtils::isBetween(const float value, const float min, const float max)
{
	return value >= min && value <= max;
}

__device__ bool CudaUtils::isBetweenExclusive(const float value, const float min, const float max)
{
	return value > min && value < max;
}

__device__ float CudaUtils::clamp(const float value, const float min, const float max)
{
	return fminf(fmaxf(min, value), max);
}

__device__ float3 CudaUtils::clamp(const float3 & value, const float3 & min, const float3 & max)
{
	float3 result = value;
	result.x = clamp(result.x, min.x, max.x);
	result.y = clamp(result.y, min.y, max.y);
	result.z = clamp(result.z, min.z, max.z);

	return result;
}

__device__ bool CudaUtils::isNan(const float3 &a)
{
	return isnan(a.x) || isnan(a.y) || isnan(a.z);
}

__device__ size_t CudaUtils::MyID()
{
	return blockIdx.x * blockDim.x + threadIdx.x;
}

__device__ float3 CudaUtils::normalize(const float3 &a)
{
	assert(!isZero(a));

	return a / norm3df(a.x, a.y, a.z);
}

__device__ float CudaUtils::len(const float3 &a)
{
	return norm3df(a.x, a.y, a.z);
}

__device__ float CudaUtils::distance(const float3 &a, const float3 &b)
{
	return norm3df(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ float CudaUtils::min3(const float a, const float b, const float c)
{
	return fminf(a, fminf(b, c));
}

__device__ float3 CudaUtils::minf3(const float3 &a, const float3 &b)
{
	return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

__device__ float3 CudaUtils::min3(const float3 &a, const float3 &b, const float3 &c)
{
	return minf3(a, minf3(b, c));
}

__device__ float CudaUtils::max3(const float a, const float b, const float c)
{
	return fmaxf(a, fmaxf(b, c));
}

__device__ float3 CudaUtils::maxf3(const float3 &a, const float3 &b)
{
	return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

__device__ float3 CudaUtils::max3(const float3 &a, const float3 &b, const float3 &c)
{
	return maxf3(a, maxf3(b, c));
}

__device__ float3 CudaUtils::cross(const float3 &a, const float3 &b)
{
	return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

__device__ float CudaUtils::dot(const float3 &a, const float3 &b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ float3 CudaUtils::reflect(const float3 & vec, const float3 & normal)
{
	return vec - 2 * dot(vec, normal) * normal;
}

__device__ void CudaUtils::tangentNormal(const float3 & vec, const float3 & normal, float3 & outTangent, float3 & outNormal)
{
	outNormal = dot(vec, normal) * normal;
	outTangent = vec - outNormal;
}

__device__ float3 CudaUtils::FaceNormal(const float3 &a, const float3 &b, const float3 &c)
{
	return CudaUtils::cross(b - a, c - a);
}

__device__ uint64_t CudaUtils::ExpandBy2(uint64_t v)
{
	v = (v * 0x000100000001u) & 0xFFFF00000000FFFFu;
	v = (v * 0x000000010001u) & 0x00FF0000FF0000FFu;
	v = (v * 0x000000000101u) & 0xF00F00F00F00F00Fu;
	v = (v * 0x000000000011u) & 0x30C30C30C30C30C3u;
	v = (v * 0x000000000005u) & 0x9249249249249249u;

	return v;
}

//__device__ uint32_t CudaUtils::ExpandBy2(uint32_t v)
//{
//	/*x &= 0x000003ff;
//	x = (x ^ (x << 16)) & 0xff0000ff;
//	x = (x ^ (x << 8)) & 0x0300f00f;
//	x = (x ^ (x << 4)) & 0x030c30c3;
//	x = (x ^ (x << 2)) & 0x09249249;
//	return x;*/
//
//	v = (v * 0x00010001u) & 0xFF0000FFu;
//	v = (v * 0x00000101u) & 0x0F00F00Fu;
//	v = (v * 0x00000011u) & 0xC30C30C3u;
//	v = (v * 0x00000005u) & 0x49249249u;
//	return v;
//}

//__device__ uint32_t CudaUtils::Morton3D(const uint32_t x, const uint32_t y, const uint32_t z)
//{
//	return (ExpandBy2(x) << 2) + (ExpandBy2(y) << 1) + ExpandBy2(z);
//}

__device__ uint64_t CudaUtils::Morton3D(const float x, const float y, const float z)
{	
	return (ExpandBy2((uint64_t) (min(max(x * 2097152.f, 0.0f), 2097151.0f))) << 2) +
		   (ExpandBy2((uint64_t) (min(max(y * 2097152.f, 0.0f), 2097151.0f))) << 1) +
		    ExpandBy2((uint64_t) (min(max(z * 2097152.f, 0.0f), 2097151.0f)));
}

//__device__ uint32_t CudaUtils::Morton3D(const float3 position)
//{
//	return Morton3D((uint32_t)position.x, (uint32_t)position.y, (uint32_t)position.z);
//}

__device__ uint64_t CudaUtils::Morton3D64(const float3 position)
{
	return Morton3D(position.x, position.y, position.z);
}


__device__ int CudaUtils::ComPref(const unsigned int mortonA, const unsigned int mortonB, const int indexA, const int indexB, const int max)
{
	if (indexB < 0 || indexB > max)
		return -1;

	unsigned int xor = mortonA ^ mortonB;
	return __clz(xor ? xor : (indexA ^ indexB));
	//return xor ? __clz(xor) : __clz(indexA ^ indexB) + 32;
}

__device__ int CudaUtils::Delta(const uint64_t * mortonCodes, const int i, const int j, const int max)
{
	if (j < 0 || j > max)
		return -1;

	uint64_t xor = mortonCodes[i] ^ mortonCodes[j];
	//return __clz(xor ? xor : (i ^ j));
	//if (!xor)
	//{
	//	printf("XOR 0, i: %d, j: %d\n", i, j);
	//}

	return xor ? __clzll(xor) : __clzll(((uint64_t) i) ^ ((uint64_t) j)) + 64;
}

__device__ int CudaUtils::sgn(const int a)
{
	//int result = ((int)(0 > a)) - ((int)(a < 0));

	int result = signbit((float)a) ? -1 : 1;

	//printf("\t\tSign of %d is %d\n", a, result);

	return result;
}

__device__ bool CudaUtils::AABBOverlap(const float3 aMin, const float3 aMax, const float3 bMin, const float3 bMax)
{
	bool a = aMax.x < bMin.x || aMin.x > bMax.x;
	bool b = aMax.y < bMin.y || aMin.y > bMax.y;
	bool c = aMax.z < bMin.z || aMin.z > bMax.z;

	return !(a || b || c);
}

__device__ void CudaUtils::PushBack(const Physics::AABBCollision & element, Physics::AABBCollision *& array, const int crtId, int & size)
{	
}

__device__ void CudaUtils::CheckResize(Physics::AABBCollision *& array, const int crtId, int & size)
{
	if (crtId / size > 0.7f)
	{
		//printf("Calling resize crt: %d, size: %d\n", crtId, size);
		Physics::AABBCollision *newArr = (Physics::AABBCollision *)malloc(2 * size * sizeof(Physics::AABBCollision));
		memcpy(newArr, array, crtId * sizeof(Physics::AABBCollision));
		size *= 2;
		free(array);
		array = newArr;
	}
}


__device__ const float3 CudaUtils::AdvancePositionInTime(const float3 & position, const float3 & velocity, const float time)
{
	return position + velocity * time;
}

__device__ const float3 CudaUtils::AdvancePositionInTimePos(const float3 & prevPosition, const float3 & position, const float time)
{
	return (1.f - time) * prevPosition + time * position;
}

__device__ void CudaUtils::ClearBit(char & byte, const int bitIndex)
{
	byte &= ~(1 << bitIndex);
}

__device__ bool CudaUtils::GetBit(const char byte, const int bitIndex)
{
	return (byte & (1 << bitIndex)) != 0;
}

__device__ void CudaUtils::SetBit(char &byte, const int bitIndex)
{
	byte |= (1 << bitIndex);
}

void CudaUtils::TempStorageGrow(void *& storage, uint64_t & size, const uint64_t requiredSize)
{
	if (requiredSize > size)
	{
		if (storage != NULL)
		{
			cudaFree(storage);
		}

		cudaMalloc(&storage, requiredSize);

		size = requiredSize;
	}
}

//template<typename T>
//T *CudaUtils::ToRaw(thrust::device_vector<T> &vec)
//{
//	return thrust::raw_pointer_cast(&vec[0]);
//}