#include "BenchmarkResultManager.h"
#include "../Benchmark/Plotter.h"
#include "../Core/Utils.hpp"

Managers::BenchmarkResultManager::BenchmarkResultManager()
{
	m_frameIndex = 0;
	m_scenarioClassIndex = 0;
	m_scenarioIndex = 0;
}

void Managers::BenchmarkResultManager::Init(size_t numberOfScenarioClasses, size_t increment, size_t numberOfFrames, size_t maximumObjects)
{
	m_numberOfScenarioClasses = numberOfScenarioClasses;
	m_objectIncrementStep = increment;
	m_numberOfFrames = numberOfFrames;
	m_maximumObjects = maximumObjects;
	m_scenariosPerClass = m_maximumObjects / m_objectIncrementStep;

	ReserveSpace();
}

void Managers::BenchmarkResultManager::ReserveSpace()
{
	m_results.clear();

	m_results.resize(m_numberOfScenarioClasses);
	for (int i = 0; i < m_results.size(); ++i)
	{
		m_results[i].resize(m_scenariosPerClass);
		for (int j = 0; j < m_results[i].size(); ++j)
		{
			m_results[i][j].resize(m_numberOfFrames);
			for (int k = 0; k < m_results[i][j].size(); ++k)
			{
				m_results[i][j][k].resize(METHOD_NUM);
				for (int l = 0; l < m_results[i][j][k].size(); ++l)
				{
					m_results[i][j][k][l].resize(CRITERIA_NUM);
				}
			}
		}
	}

	m_averageResults.clear();

	m_averageResults.resize(m_numberOfScenarioClasses);
	for (int i = 0; i < m_averageResults.size(); ++i)
	{
		m_averageResults[i].resize(m_scenariosPerClass);
		for (int j = 0; j < m_averageResults[i].size(); ++j)
		{
			m_averageResults[i][j].resize(METHOD_NUM);
			for (int k = 0; k < m_averageResults[i][j].size(); ++k)
			{
				m_averageResults[i][j][k].resize(CRITERIA_NUM);
			}
		}
	}
}

void Managers::BenchmarkResultManager::ScenarioClassEnded()
{
	m_scenarioClassIndex++;
	ResetScenario();
	ResetFrame();
}

void Managers::BenchmarkResultManager::ScenarioEnded()
{
	m_scenarioIndex++;
	ResetFrame();
}

void Managers::BenchmarkResultManager::FrameEnded()
{
	m_frameIndex++;
}

void Managers::BenchmarkResultManager::ResetScenarioClass()
{
	m_scenarioClassIndex = 0;
	ResetScenario();
	ResetFrame();
}

void Managers::BenchmarkResultManager::ResetScenario()
{
	m_scenarioIndex = 0;
	ResetFrame();
}

void Managers::BenchmarkResultManager::ResetFrame()
{
	m_frameIndex = 0;
}

void Managers::BenchmarkResultManager::RecordCriterion(std::wstring  methodName, std::wstring  criterionName, float value)
{
	//0 4 0 1 1
	m_results[m_scenarioClassIndex][m_scenarioIndex][m_frameIndex][m_nameToIndexTable.at(methodName)][m_nameToIndexTable.at(criterionName) - METHOD_NUM - 1] = value;
	m_averageResults[m_scenarioClassIndex][m_scenarioIndex][m_nameToIndexTable.at(methodName)][m_nameToIndexTable.at(criterionName) - METHOD_NUM - 1] += value / m_numberOfFrames;
}

void Managers::BenchmarkResultManager::DumpToDisk()
{
	for (int i = 0; i < m_numberOfScenarioClasses; ++i)
	{
		for (int j = 0; j < m_scenariosPerClass; ++j)
		{
			for (int k = 0; k < m_numberOfFrames; ++k)
			{
				for (int l = 0; l < METHOD_NUM; ++l)
				{
					m_results[i][j][k][l][INDEX_TOTAL_TIME - METHOD_NUM - 1] = m_results[i][j][k][l][INDEX_UPDATE_TIME - METHOD_NUM - 1] + m_results[i][j][k][l][INDEX_COLLISION_TIME - METHOD_NUM - 1];
					m_averageResults[i][j][l][INDEX_TOTAL_TIME - METHOD_NUM - 1] = m_averageResults[i][j][l][INDEX_UPDATE_TIME - METHOD_NUM - 1] + m_averageResults[i][j][l][INDEX_COLLISION_TIME - METHOD_NUM - 1];
				}
			}
		}
	}

	for (int i = 0; i < m_numberOfScenarioClasses; ++i)
	{		
		for (int k = 0; k < METHOD_NUM; ++k)
		{
			auto file = OpenScenarioFile(i, m_indexToNameTable.at(k));

			file << "[Frame] ";
			for (int m = METHOD_NUM + 1; m < CRITERIA_NUM + METHOD_NUM + 1; ++m)
			{
				std::string asd(m_indexToNameTable.at(m).begin(), m_indexToNameTable.at(m).end());

				file << "[" << asd << "] ";
			}

			file << std::endl;

			for (int l = 0; l < m_numberOfFrames; ++l)
			{
				file << l << " ";
				for (int m = 0; m < CRITERIA_NUM; ++m)
				{					
					file << m_results[i][m_scenariosPerClass - 1][l][k][m] << " ";					
				}
				file << std::endl;
			}

			file.close();
		}
	}

	for (int i = 0; i < m_numberOfScenarioClasses; ++i)
	{
		for (int j = 0; j < METHOD_NUM; ++j)
		{
			auto file = OpenAverageScenarioFile(i, m_indexToNameTable.at(j));

			file << "[Objects] ";
			for (int m = METHOD_NUM + 1; m < CRITERIA_NUM + METHOD_NUM + 1; ++m)
			{
				std::string asd(m_indexToNameTable.at(m).begin(), m_indexToNameTable.at(m).end());

				file << "[Average " << asd << "] ";
			}

			file << std::endl;
			for (int k = 0; k < m_scenariosPerClass; ++k)
			{
				file << (k + 1) * m_objectIncrementStep << " ";
				for (int l = 0; l < CRITERIA_NUM; ++l)
				{
					file << m_averageResults[i][k][j][l] << " ";
				}
				file << std::endl;
			}
			file.close();
		}
	}

	Core::Utils::GeneratePlotsFromRawData();
	ResetScenarioClass();
	ReserveSpace();
}

std::ofstream Managers::BenchmarkResultManager::OpenScenarioFile(int index, std::wstring  methodName)
{
	//std::wstring, wchar_t, std::char_traits<wchar_t>
	std::ofstream file;

	file.open(Core::RAW_RESULT_FOLDER + L"Scenario_" + std::to_wstring(index) + L"_" + methodName + L".txt", std::ios::out | std::ios::trunc);


	if (!file.is_open())
	{
		std::wcout << L"ERROR opening file\n";
	}

	return file;
}

std::ofstream Managers::BenchmarkResultManager::OpenAverageScenarioFile(int index, std::wstring  methodName)
{
	std::ofstream file;

	file.open(Core::RAW_RESULT_FOLDER + L"Scenario_" + std::to_wstring(index) + L"_average_" + methodName + L".txt", std::ios::out | std::ios::trunc);


	if (!file.is_open())
	{
		std::wcout << "ERROR opening file\n";
	}

	return file;
}
