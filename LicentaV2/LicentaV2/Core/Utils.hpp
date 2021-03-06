#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <Windows.h>
#include <ctime>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/freeglut/freeglut.h"
/*#include <../../../../../../../../Program Files (x86)/Windows Kits/10/Include/10.0.10240.0/ucrt/tchar.h>*/


namespace Core
{

	const float TIME_STEP_MS = 1000.f / 60.f;
	const float TIME_STEP = TIME_STEP_MS / 1000.f;
	const float PHYSICS_TIME_STEP_MS = 1000.f / 60.f;
	const float PHYSICS_TIME_STEP = PHYSICS_TIME_STEP_MS / 1000.f;

	const glm::vec3 GRAVITY_ACCEL(0, -0.981f, 0);
	

	const glm::vec4 DEFAULT_OBJECT_COLOR(0.3f, 0.3f, 0.3f, 1.f);
	const glm::vec4 COLLIDING_OBJECT_COLOR(1.f, 0.f, 0.f, 1.f);
	const glm::vec4 SELF_COLLIDING_OBJECT_COLOR(0.f, 0.5f, 0.f, 1.f);
	const glm::vec4 BOUNDING_VOLUME_COLOR(0.f, 0.f, 1.f, 1.f);
	//const glm::vec4 BACKGROUND_COLOR(0.2f, 0.2f, 0.2f, 1.0f);
	const glm::vec4 BACKGROUND_COLOR(0.8f, 0.8f, 0.8f, 1.f);
	const std::string  BENCHMARK_FOLDER("BenchmarkResults/");
	const std::string  RAW_RESULT_FOLDER(BENCHMARK_FOLDER + "RawResults/");
	const std::string  PLOTS_FOLDER(BENCHMARK_FOLDER + "Plots/");
	const std::string  PER_FRAME_PLOTS_FOLDER(PLOTS_FOLDER + "PerFrame/");
	const std::string  PER_SCENARIO_PLOTS_FOLDER(PLOTS_FOLDER + "PerScenario/");

	const size_t SCENARIO_CLASSES = 5;
	const size_t FRAMES_NUM = 300;
	const size_t OBJECT_INCREMENT = 50;
	const size_t MAX_NUMBER_OBJECTS = OBJECT_INCREMENT * 2;
	const bool REPLAY_SCENARIO = false;

	const std::string  STRUCTURE_TIME("Time Spent - Structure Update");
	const std::string  COLLISION_TIME("Time Spent - Collisions");
	const std::string  TOTAL_TIME("Time Spent - Total");
	const std::string  INTERSECTION_TESTS("Intersection Tests");
	const std::string  MEMORY("Memory Used (KB)");

	const std::string  METHOD_NONE("None");
	const std::string  METHOD_BVH("BVH");
	const std::string  METHOD_OCTREE("Octree");
	const std::string  METHOD_SPATIAL_GRID("Spatial-Grid");
	const std::string  METHOD_SPATIAL_GRID_OPTIMIZED("Spatial-Grid-Optimized");
	const std::string  METHOD_SPATIAL_HASHING("Spatial-Hashing");
	const std::string  METHOD_SAP("Sweep-and-Prune");
	const std::string  METHOD_S2S("Sphere-to-Sphere");





	class Utils
	{
	public:

		static void GeneratePlotsFromRawData()
		{
			SetCurrentDirectory(Core::BENCHMARK_FOLDER.c_str());
			system("gnuplot script.plt");
			system("gnuplot script2.plt");
			SetCurrentDirectory("..\\");
		}

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


// 			std::string asd(str.begin(), str.end());
// 			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 			glMatrixMode(GL_PROJECTION);
// 			glLoadIdentity();
// 
// 			glMatrixMode(GL_PROJECTION);
// 			glPushMatrix();
// 			glLoadIdentity();
// 			gluOrtho2D(0, 1600, 900, 0);
// 			glRasterPos2i(pos.x, pos.y);
// 			glutBitmapString(GLUT_BITMAP_9_BY_15, (const unsigned char*) asd.c_str());
// 			glPopMatrix();
		}

		static wchar_t *convertCharArrayToLPCWSTR(const char* charArray)
		{
			wchar_t* wString = new wchar_t[4096];
			MultiByteToWideChar(CP_ACP, 0, charArray, -1, wString, 4096);
			return wString;
		}

		static std::vector<std::string> GetAllFilesInFolder(std::string folder)
		{
			std::vector<std::string> names;
			//TCHAR search_path[200];
			std::string  searchPath = folder + "\\*.txt";
			//_tprintf(search_path, 200, "%s/*.txt", folder.c_str());
			WIN32_FIND_DATA fd;

			auto str = searchPath.c_str();
			HANDLE hFind = ::FindFirstFile(str, &fd);
			if (hFind != INVALID_HANDLE_VALUE) {
				do {
					if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
						names.push_back(std::string(fd.cFileName));
					}
				} while (::FindNextFile(hFind, &fd));
				::FindClose(hFind);
			}

			//delete str;
			return names;
		}

		static float RandomRange(float min, float max)
		{
			return max - min == 0 ? min : min + (float)(std::rand()) / (float(RAND_MAX / (max - min)));
		}

		static float RandomAround(const float center, const float radius)
		{
			return RandomRange(center - radius, center + radius);
		}

		static glm::vec3 RandomVec3Around(const glm::vec3 &position, const float radius)
		{
			return glm::vec3
			(
				RandomAround(position.x, radius),
				RandomAround(position.y, radius),
				RandomAround(position.z, radius)
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

		static glm::vec3 RandomRangeVec(float min, float max)
		{
			return glm::vec3(RandomRange(min, max),	RandomRange(min, max), RandomRange(min, max));
		}

		static float scaleValue(const float val, std::pair<float, float> oldRange, std::pair<float, float> newRange)
		{
			return ((newRange.second - newRange.first) * (val - oldRange.first) / (oldRange.second - oldRange.first)) + newRange.first;
		}

		static float Clamp(const float val, const float lo, const float hi)
		{
			return val < lo ? lo : val > hi ? hi : val;
		}

		static glm::vec3 ProjectPointLine(glm::vec3 point, glm::vec3 edgeP1, glm::vec3 edgeP2)
		{
			glm::vec3 ab = edgeP2 - edgeP1;
			glm::vec3 ap = point - edgeP1;

			return edgeP1 + glm::dot(ap, ab) / glm::dot(ab, ab) * ab;
		}
	};
}
