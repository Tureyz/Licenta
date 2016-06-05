#pragma once
#include <vector>

namespace Benchmark
{
	class MemoryCounter
	{
	public:
		MemoryCounter();
		~MemoryCounter();

		void addDynamic(size_t mem);				
		void subDynamic(size_t mem);
		void addLocal(size_t mem);
		void subLocal(size_t mem);
		void resetAll();
		void clearDynamic();
		size_t GetMaxMemory() const { return m_maxMemory; }
		void SetMaxMemory(size_t val) { m_maxMemory = val; }
		size_t GetMinMemory() const { return m_minMemory; }
		void SetMinMemory(size_t val) { m_minMemory = val; }

		void pushScope();
		void popScope();

		void computeLimits();
	private:
			size_t m_currentMemory;
			size_t m_maxMemory;
			size_t m_minMemory;
			size_t m_currentLocal;

			std::vector<size_t> m_scopes;
	};
}