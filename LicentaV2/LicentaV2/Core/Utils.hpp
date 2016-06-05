#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <Windows.h>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/freeglut/freeglut.h"


namespace Core
{
	const glm::vec4 defaultObjectColor(0.7f, 0.7f, 0.7f, 1.f);
	const std::string benchmarkFolder("BenchmarkResults/");
	const std::string rawResultFolder(benchmarkFolder + "RawResults/");
	const std::string plotsFolder(benchmarkFolder + "Plots/");

	class Utils
	{
	public:
		static void printToScreen(glm::vec2 pos, std::string str)
		{
			//TODO find stuff that's not deprecated

			//glDisable(GL_TEXTURE_2D);
			//glColor4f(0, 0, 1, 1);
			//glRasterPos2i(pos.x, pos.y);
			//glWindowPos3f(pos.x, pos.y, 0);
			//glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*) str.c_str());
			//std::cout << str << std::endl;
			//glEnable(GL_TEXTURE_2D);
		}

		static std::vector<std::string> GetAllFilesInFolder(std::string folder)
		{
			std::vector<std::string> names;
			char search_path[200];
			sprintf_s(search_path, 200, "%s/*.txt", folder.c_str());
			WIN32_FIND_DATA fd;
			HANDLE hFind = ::FindFirstFile(search_path, &fd);
			if (hFind != INVALID_HANDLE_VALUE) {
				do {
					if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
						names.push_back(fd.cFileName);
					}
				} while (::FindNextFile(hFind, &fd));
				::FindClose(hFind);
			}
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