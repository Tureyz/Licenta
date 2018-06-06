#pragma once
#include "../Core/CudaUtils.cuh"
#include <map>

#define _BENCHMARK_ON

class GPUBenchmark
{
public:
	//GPUBenchmark() {}
	GPUBenchmark(const uint64_t sample, const double freeVRAMInit);

	void Start();
	void Record(const std::string name);
	void Step();

private:
	std::map<std::string, float3> m_map;
	uint64_t m_sample;
	uint64_t m_crtStep;

	double m_freeVRAMInit;

	CudaUtils::CudaTimer m_timer;
};
