#pragma once
#include <iostream>
#include "../../Dependencies/glew/glew.h"
#include "../../Dependencies/freeglut/freeglut.h"
namespace Core {
	namespace Init {

		class InitGLEW
		{
		public:
			static void Init();
		};
	}
}