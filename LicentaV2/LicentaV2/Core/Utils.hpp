#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <Windows.h>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/freeglut/freeglut.h"
#include <../../../../../../../../Program Files (x86)/Windows Kits/10/Include/10.0.10240.0/ucrt/tchar.h>

namespace Core
{
	const glm::vec4 DEFAULT_OBJECT_COLOR(0.2f, 0.2f, 0.2f, 1.f);
	//const glm::vec4 BACKGROUND_COLOR(0.2f, 0.2f, 0.2f, 1.0f);
	const glm::vec4 BACKGROUND_COLOR(0.8f, 0.8f, 0.8f, 1.f);
	const std::wstring  BENCHMARK_FOLDER(L"BenchmarkResults/");
	const std::wstring  RAW_RESULT_FOLDER(BENCHMARK_FOLDER + L"RawResults/");
	const std::wstring  PLOTS_FOLDER(BENCHMARK_FOLDER + L"Plots/");
	const std::wstring  PER_FRAME_PLOTS_FOLDER(PLOTS_FOLDER + L"PerFrame/");
	const std::wstring  PER_SCENARIO_PLOTS_FOLDER(PLOTS_FOLDER + L"PerScenario/");

	const size_t SCENARIO_CLASSES = 5;
	const size_t MAX_NUMBER_OBJECTS = 30;
	const size_t FRAMES_NUM = 300;
	const size_t OBJECT_INCREMENT = 15;
	const bool REPLAY_SCENARIO = false;

	const std::wstring  STRUCTURE_TIME(L"Time Spent - Structure Update");
	const std::wstring  COLLISION_TIME(L"Time Spent - Collisions");
	const std::wstring  TOTAL_TIME(L"Time Spent - Total");
	const std::wstring  INTERSECTION_TESTS(L"Intersection Tests");
	const std::wstring  MEMORY(L"Memory Used (KB)");

	const std::wstring  METHOD_NONE(L"None");
	const std::wstring  METHOD_BVH(L"BVH");
	const std::wstring  METHOD_OCTREE(L"Octree");
	const std::wstring  METHOD_SPATIAL_GRID(L"Spatial-Grid");
	const std::wstring  METHOD_SPATIAL_GRID_OPTIMIZED(L"Spatial-Grid-Optimized");
	const std::wstring  METHOD_SPATIAL_HASHING(L"Spatial-Hashing");
	const std::wstring  METHOD_SAP(L"Sweep-and-Prune");
	const std::wstring  METHOD_S2S(L"Sphere-to-Sphere");

	class Utils
	{
	public:
		static void printToScreen(glm::vec2 pos, std::wstring  str)
		{
			//TODO find stuff that's not deprecated

			//glDisable(GL_TEXTURE_2D);
			//glColor4f(0, 0, 1, 1);
			//glRasterPos2i(pos.x, pos.y);
			//glWindowPos3f(pos.x, pos.y, 0);
			//glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*) str.c_str());
			//std::wcout << str << std::endl;
			//glEnable(GL_TEXTURE_2D);
		}

		static wchar_t *convertCharArrayToLPCWSTR(const char* charArray)
		{
			wchar_t* wString = new wchar_t[4096];
			MultiByteToWideChar(CP_ACP, 0, charArray, -1, wString, 4096);
			return wString;
		}

		static std::vector<std::wstring> GetAllFilesInFolder(std::wstring  folder)
		{
			std::vector<std::wstring> names;
			//TCHAR search_path[200];
			std::wstring  searchPath = folder + L"\\*.txt";
			//_tprintf(search_path, 200, "%s/*.txt", folder.c_str());
			WIN32_FIND_DATA fd;

			auto str = searchPath.c_str();
			HANDLE hFind = ::FindFirstFile(str, &fd);
			if (hFind != INVALID_HANDLE_VALUE) {
				do {
					if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
						names.push_back(std::wstring(fd.cFileName));
					}
				} while (::FindNextFile(hFind, &fd));
				::FindClose(hFind);
			}

			//delete str;
			return names;
		}

		static glm::vec3 RandomVec3Around(const glm::vec3 &position, const float radius)
		{
			return glm::vec3
			(
				position.x - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius))),
				position.y - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius))),
				position.z - radius + (float)(std::rand()) / (float(RAND_MAX / (2 * radius)))
			);
		}

		static glm::vec3 Random01()
		{
			return glm::vec3
			(
				(std::rand() % 1000) / 1000.f,
				(std::rand() % 1000) / 1000.f,
				(std::rand() % 1000) / 1000.f
			);
		}

		static glm::vec3 RandomRange(float min, float max)
		{
			return glm::vec3
			(
				min + (float)(std::rand()) / (float(RAND_MAX / (max - min))),
				min + (float)(std::rand()) / (float(RAND_MAX / (max - min))),
				min + (float)(std::rand()) / (float(RAND_MAX / (max - min)))
			);
		}		
	};
}