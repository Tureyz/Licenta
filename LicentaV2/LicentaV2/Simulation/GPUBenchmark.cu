#include "GPUBenchmark.cuh"

GPUBenchmark::GPUBenchmark(const uint64_t sample, const double freeVRAMInit)
{
#ifdef _BENCHMARK_ON
	m_sample = sample;
	m_crtStep = 0;
	m_freeVRAMInit = freeVRAMInit;
	m_timer = CudaUtils::CudaTimer();
#endif
}

void GPUBenchmark::Start()
{
#ifdef _BENCHMARK_ON
	m_timer.Start();
#endif
}

void GPUBenchmark::Record(const std::string name)
{
#ifdef _BENCHMARK_ON
	float crtTimeMS = m_timer.End();

	if (!m_map.count(name))
	{
		m_map[name] = make_float3(crtTimeMS, crtTimeMS, crtTimeMS);
	}
	else
	{
		float3 minMaxTotal = m_map.at(name);
		minMaxTotal.x = crtTimeMS < minMaxTotal.x ? crtTimeMS : minMaxTotal.x;
		minMaxTotal.x = crtTimeMS > minMaxTotal.y ? crtTimeMS : minMaxTotal.y;
		minMaxTotal.z += crtTimeMS;		
	}
#endif
}

void GPUBenchmark::Step()
{
#ifdef _BENCHMARK_ON
	m_crtStep++;

	if (m_crtStep % m_sample == 0)
	{
		std::cout << "Times after " << m_sample << " steps: " << std::endl;

		for (auto kvPair : m_map)
		{
			std::string name = kvPair.first;
			float3 minMaxTotal = kvPair.second;
			std::cout << name << "-> min: " << minMaxTotal.x << "ms, max: " << minMaxTotal.y << "ms, avg: " << minMaxTotal.z / m_crtStep << "ms" << std::endl;
		}

		CudaUtils::MemUsage(m_freeVRAMInit);
		std::cout << "----------------------------------------------------" << std::endl;
	}
#endif
}
