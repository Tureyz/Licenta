#pragma once
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include "../Core/Utils.hpp"

namespace Managers
{
	class BenchmarkResultManager
	{
	public:
		BenchmarkResultManager();
		enum methodIndices { INDEX_BVH = 0, INDEX_OCTREE = 1, INDEX_SPATIAL_GRID = 2, INDEX_SPATIAL_GRID_OPTIMIZED = 3, INDEX_SPATIAL_HASHING = 4, INDEX_SAP = 5, INDEX_NONE = 6, METHOD_NUM = 7 };
		enum criteriaIndices { INDEX_COLLISION_TIME = 8, INDEX_UPDATE_TIME = 9, INDEX_TOTAL_TIME = 10, INDEX_TESTS = 11, INDEX_MEMORY = 12, CRITERIA_NUM = 5 };

		const std::unordered_map<std::wstring , int> m_nameToIndexTable = {
			{ Core::METHOD_NONE, INDEX_NONE },
			{ Core::METHOD_BVH, INDEX_BVH },
			{ Core::METHOD_OCTREE, INDEX_OCTREE },
			{ Core::METHOD_SPATIAL_GRID, INDEX_SPATIAL_GRID },
			{ Core::METHOD_SPATIAL_GRID_OPTIMIZED, INDEX_SPATIAL_GRID_OPTIMIZED },
			{ Core::METHOD_SPATIAL_HASHING, INDEX_SPATIAL_HASHING },
			{ Core::METHOD_SAP, INDEX_SAP },
			{ Core::COLLISION_TIME, INDEX_COLLISION_TIME }, 
			{ Core::STRUCTURE_TIME, INDEX_UPDATE_TIME },
			{ Core::TOTAL_TIME, INDEX_TOTAL_TIME },
			{ Core::INTERSECTION_TESTS, INDEX_TESTS },
			{ Core::MEMORY, INDEX_MEMORY }
		};

		const std::unordered_map<int, std::wstring > m_indexToNameTable = {
			{ INDEX_NONE, Core::METHOD_NONE },
			{ INDEX_BVH, Core::METHOD_BVH },
			{ INDEX_OCTREE, Core::METHOD_OCTREE },
			{ INDEX_SPATIAL_GRID, Core::METHOD_SPATIAL_GRID },
			{ INDEX_SPATIAL_GRID_OPTIMIZED, Core::METHOD_SPATIAL_GRID_OPTIMIZED },
			{ INDEX_SPATIAL_HASHING, Core::METHOD_SPATIAL_HASHING },
			{ INDEX_SAP, Core::METHOD_SAP },
			{ INDEX_COLLISION_TIME, Core::COLLISION_TIME },
			{ INDEX_UPDATE_TIME, Core::STRUCTURE_TIME },
			{ INDEX_TOTAL_TIME, Core::TOTAL_TIME },
			{ INDEX_TESTS, Core::INTERSECTION_TESTS },
			{ INDEX_MEMORY, Core::MEMORY }
		};

		//first dimension = number of scenarios classes
		//second dimension = number of objects in current scenario
		//third dimension = frame number
		//fourth dimension = method
		//fifth dimension = criterion
		std::vector<std::vector<std::vector<std::vector<std::vector<float>>>>> m_results;

		//first dimension = number of scenarios classes
		//second dimension = number of objects in current scenario		
		//third dimension = method
		//fourth dimension = criterion
		std::vector<std::vector<std::vector<std::vector<float>>>> m_averageResults;

		void Init(size_t numberOfScenarioClasses, size_t increment, size_t numberOfFrames, size_t maximumObjects);
		void ReserveSpace();

		void ScenarioClassEnded();
		void ScenarioEnded();
		void FrameEnded();
		void ResetScenarioClass();
		void ResetScenario();
		void ResetFrame();

		void RecordCriterion(std::wstring  methodName, std::wstring  criterionName, float value);
		void DumpToDisk();

		std::ofstream OpenScenarioFile(int index, std::wstring  methodName);
		std::ofstream OpenAverageScenarioFile(int index, std::wstring  methodName);
		size_t m_objectIncrementStep;
		size_t m_maximumObjects;
		size_t m_numberOfScenarioClasses;
		size_t m_scenariosPerClass;
		size_t m_numberOfFrames;

		int m_scenarioClassIndex;
		int m_scenarioIndex;
		int m_frameIndex;
	};
}
