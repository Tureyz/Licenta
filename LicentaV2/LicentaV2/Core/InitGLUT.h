#pragma once
#include "../Managers/SceneManager.h"
#include <iostream>


namespace Core {
	namespace Init {
		const std::wstring  windowName = L"Hurezeanu App";
		const glm::vec2 defaultWindowSize(1600, 900);
		const glm::vec2 defaultWindowPos(0, 0);
		const int openGLMajorVersion = 4;
		const int openGLMinorVersion = 5;
			
		class InitGLUT {
		public:
			static void InitGLUTFunc(int argc, char **argv);

			static void Run();
			static void Close();

			void EnterFullScreen();
			void ExitFullScreen();

			static void SetSceneManager(Managers::SceneManager *&sceneManagerArg);

		private:			
			static Managers::SceneManager *sceneManager;

			static void IdleCallback(void);
			static void DisplayCallback(void);
			static void ReshapeCallback(int width, int height);
			static void CloseCallback();
			static void KeyboardDownCallback(unsigned char key, int x, int y);
			static void KeyboardUpCallback(unsigned char key, int x, int y);
			static void MouseCallback(int button, int state, int x, int y);
			static void MotionCallback(int x, int y);
		};
	}
}