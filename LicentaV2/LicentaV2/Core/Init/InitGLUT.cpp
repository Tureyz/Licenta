#include "InitGLUT.h"
#include "ListenerInterface.h"
#include "DebugOutput.h"

Managers::SceneManager* Core::Init::InitGLUT::listener = NULL;
Core::WindowProperties Core::Init::InitGLUT::windowProperties;
//int m_frame = 0, m_time, m_timebase = 0;

void Core::Init::InitGLUT::InitGLUTFunc(const Core::WindowProperties & windowProps, const Core::ContextProperties & contextProperties, const Core::FrameBufferProperties & framebufferProperties, int argc, char * argv[])
{
	windowProperties = windowProps;
	glutInit(&argc, argv);

	if (contextProperties.m_core)
	{
		glutInitContextVersion(contextProperties.m_majorVersion, contextProperties.m_minorVersion);
		glutInitContextProfile(GLUT_CORE_PROFILE);
	}
	else
	{
		glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);
	}

	glutInitDisplayMode(framebufferProperties.m_flags);
	glutInitWindowPosition((int) windowProps.m_pos.x, (int)windowProps.m_pos.y);
	glutInitWindowSize((int) windowProps.m_size.x, (int) windowProps.m_size.y);

	glutCreateWindow(windowProps.m_name.c_str());

	std::cout << "GLUT:initialized" << std::endl;
	//glEnable(GL_DEBUG_OUTPUT);

	glutIdleFunc(IdleCallback);
	glutCloseFunc(CloseCallback);
	glutDisplayFunc(DisplayCallback);
	glutReshapeFunc(ReshapeCallback);

	glutKeyboardFunc(KeyboardCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);

	Init::InitGLEW::Init();
	//glDebugMessageCallback(DebugOutput::myCallback, NULL);
	//glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	PrintOpenGLInfo(windowProps, contextProperties);
	//m_frame = 0;
	//m_timebase = 0;
}

void Core::Init::InitGLUT::Run()
{
	std::cout << "GLUT:\t Started Running " << std::endl;
	glutMainLoop();
}

void Core::Init::InitGLUT::Close()
{
	std::cout << "GLUT:\t Finished" << std::endl;
	glutLeaveMainLoop();
}

void Core::Init::InitGLUT::EnterFullScreen()
{
	glutFullScreen();
}

void Core::Init::InitGLUT::ExitFullScreen()
{
	glutLeaveFullScreen();
}

void Core::Init::InitGLUT::PrintOpenGLInfo(const Core::WindowProperties & windowInfo, const Core::ContextProperties & context)
{
	std::cout << "==================================================\n";
	std::cout << "GLUT:Initialized" << std::endl;
	std::cout << "GLUT:\tVendor : " << std::string((const char *)glGetString(GL_VENDOR)) << std::endl;
	std::cout << "GLUT:\tRenderer : " << std::string((const char *)glGetString(GL_RENDERER)) << std::endl;
	std::cout << "GLUT:\tOpenGl version: " << std::string((const char *)glGetString(GL_VERSION)) << std::endl;
	std::cout << "==================================================\n";
}

void Core::Init::InitGLUT::SetListener(Managers::SceneManager *&listenerInterface)
{
	listener = listenerInterface;
	listener->SetScreenProps((int) windowProperties.m_size.x, (int) windowProperties.m_size.y);
}

void Core::Init::InitGLUT::IdleCallback(void)
{
	glutPostRedisplay();
// 
// 	m_frame++;
// 	m_time = glutGet(GLUT_ELAPSED_TIME);
// 
// 	if (m_time - m_timebase > 1000) {
// 		float fps = m_frame * 1000.0 / (m_time - m_timebase);
// 		std::cout << "FPS: " << fps << std::endl;
// 		m_timebase = m_time;
// 		m_frame = 0;
// 	}

}

void Core::Init::InitGLUT::DisplayCallback(void)
{
	if (listener)
	{
		listener->notifyBeginFrame();
		listener->notifyDisplayFrame();

		glutSwapBuffers();

		listener->notifyEndFrame();
	}
}

void Core::Init::InitGLUT::ReshapeCallback(int width, int height)
{
	if (windowProperties.m_isReshapable == true)
	{
		if (listener)
		{
			listener->notifyReshape(width, height, (int) windowProperties.m_size.x, (int) windowProperties.m_size.y);
			listener->SetScreenProps(width, height);
			std::cout << "GLUT: \tReshaped\n";
		}
		windowProperties.m_size = glm::vec2(width, height);
		glViewport(0, 0, width, height);
	}
}

void Core::Init::InitGLUT::CloseCallback()
{
	Close();
}

void Core::Init::InitGLUT::KeyboardCallback(unsigned char key, int x, int y)
{
	listener->KeyboardCallback(key, x, y);
}

void Core::Init::InitGLUT::MouseCallback(int button, int state, int x, int y)
{
	listener->MouseCallback(button, state, x, y);
}

void Core::Init::InitGLUT::MotionCallback(int x, int y)
{
	listener->MotionCallback(x, y);
}
