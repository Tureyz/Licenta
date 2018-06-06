#include "GPUBenchmark.cuh"
#include <iomanip>

GPUBenchmark::GPUBenchmark(const uint64_t sample, const double freeVRAMInit)
{
#ifdef _BENCHMARK_ON
	m_sample = sample;
	m_crtStep = 0;
	m_freeVRAMInit = freeVRAMInit;
	//m_timer = CudaUtils::CudaTimer();
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
		minMaxTotal.y = crtTimeMS > minMaxTotal.y ? crtTimeMS : minMaxTotal.y;
		minMaxTotal.z += crtTimeMS;
		m_map[name] = minMaxTotal;
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

		float total = 0.f;
		int longestString = 0;		
		for (auto kvPair : m_map)
		{
			total += kvPair.second.z;
			longestString = kvPair.first.length() > longestString ? kvPair.first.length() : longestString;			
		}

		for (auto kvPair : m_map)
		{
			std::string name = kvPair.first;
			float3 minMaxTotal = kvPair.second;
			std::cout << std::setw(longestString + 1) << std::left << name
				<< "| min: " << std::setprecision(3) << std::fixed << std::setw(8) << minMaxTotal.x << "ms | "
				<< "max: " << std::setprecision(3) << std::fixed << std::setw(8) << minMaxTotal.y << "ms | "
				<< "avg: " << std::setprecision(3) << std::fixed << std::setw(8) << minMaxTotal.z / m_crtStep << "ms "
				<< "(" << std::setprecision(3) << std::fixed << (minMaxTotal.z / total) * 100.f << "%)" << std::endl;
		}

		std::cout << CudaUtils::MemUsage(m_freeVRAMInit) << std::endl;
		std::cout << "----------------------------------------------------" << std::endl;

		m_map.clear();
		m_crtStep = 0;
	}
#endif
}
