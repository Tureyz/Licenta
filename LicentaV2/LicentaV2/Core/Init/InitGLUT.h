#pragma once
#include "ContextProperties.h"
#include "FrameBufferProperties.h"
#include "WindowProperties.h"
#include "InitGLEW.h"
#include "../../Managers/SceneManager.h"
#include <iostream>


namespace Core {
	namespace Init {//two namespaces

		class InitGLUT {
		public:
			static void InitGLUTFunc(const Core::WindowProperties& windowProperties, const Core::ContextProperties& contextProperties,
				const Core::FrameBufferProperties& framebufferProperties, int argc, char *argv[]);

			static void Run();
			static void Close();

			void EnterFullScreen();
			void ExitFullScreen();

			//used to print info about GL
			static void PrintOpenGLInfo(const Core::WindowProperties &windowInfo, const Core::ContextProperties &context);
			static void SetListener(Managers::SceneManager *&listenerInterface);

		private:			
			static Managers::SceneManager *listener;
			static Core::WindowProperties windowProperties;

			static void IdleCallback(void);
			static void DisplayCallback(void);
			static void ReshapeCallback(int width, int height);
			static void CloseCallback();
			static void KeyboardCallback(unsigned char key, int x, int y);
			static void MouseCallback(int button, int state, int x, int y);
			static void MotionCallback(int x, int y);

			static int m_frame, m_time, m_timebase;
		};
	}
}