#pragma once
#include <string>
#include <iostream>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/freeglut/freeglut.h"

namespace Core
{
	class Utils
	{
	public:

		static void printToScreen(glm::vec2 pos, std::string str)
		{
			glDisable(GL_TEXTURE_2D);
			glColor3f(1, 1, 1);
			glWindowPos3f(pos.x, pos.y, 0);
			glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*) str.c_str());
			//std::cout << str << std::endl;
			glEnable(GL_TEXTURE_2D);
		}
	};
}