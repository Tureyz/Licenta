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
			//TODO find stuff that's not deprecated

			//glDisable(GL_TEXTURE_2D);
			//glColor4f(0, 0, 1, 1);
			//glRasterPos2i(pos.x, pos.y);
			//glWindowPos3f(pos.x, pos.y, 0);
			//glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*) str.c_str());
			//std::cout << str << std::endl;
			//glEnable(GL_TEXTURE_2D);
		}
	};
}