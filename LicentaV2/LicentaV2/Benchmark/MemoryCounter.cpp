#include "MemoryCounter.h"
#include <limits.h>

Benchmark::MemoryCounter::MemoryCounter()
{
	m_currentMemory = m_maxMemory = 0;
	m_currentLocal = 0;
	m_minMemory = INT_MAX;
}

Benchmark::MemoryCounter::~MemoryCounter()
{

}

void Benchmark::MemoryCounter::addDynamic(size_t mem)
{
	m_currentMemory += mem;

	computeLimits();
}

void Benchmark::MemoryCounter::subDynamic(size_t mem)
{
	m_currentMemory -= mem;

	//computeLimits();
}

void Benchmark::MemoryCounter::addLocal(size_t mem)
{
	m_scopes[m_scopes.size() - 1] += mem;

	m_currentLocal += mem;
	computeLimits();
}

void Benchmark::MemoryCounter::subLocal(size_t mem)
{
	m_scopes[m_scopes.size() - 1] -= mem;
	m_currentLocal -= mem;
	//computeLimits();
}

void Benchmark::MemoryCounter::resetAll()
{
	m_currentMemory = m_maxMemory = 0;
	m_currentLocal = 0;
	m_minMemory = INT_MAX;
	m_scopes.clear();
}

void Benchmark::MemoryCounter::clearDynamic()
{
	m_currentMemory = 0;

	computeLimits();
}

void Benchmark::MemoryCounter::pushScope()
{
	m_scopes.push_back(0);
}

void Benchmark::MemoryCounter::popScope()
{
	m_currentLocal -= m_scopes.back();
	m_scopes.pop_back();
	
	computeLimits();
}

void Benchmark::MemoryCounter::computeLimits()
{

	size_t total = m_currentMemory + m_currentLocal;

	if (total > m_maxMemory)
	{
		m_maxMemory = total;
	}
	if (total < m_minMemory)
	{
		m_minMemory = total;
	}
}
