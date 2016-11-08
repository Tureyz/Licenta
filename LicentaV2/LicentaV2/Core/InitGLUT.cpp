#include "InitGLUT.h"

Managers::SceneManager* Core::Init::InitGLUT::sceneManager = NULL;

static glm::vec2 windowPos;
static glm::vec2 windowSize;

void Core::Init::InitGLUT::InitGLUTFunc(int argc, char **argv)
{
	windowSize = Core::Init::defaultWindowSize;
	windowPos = Core::Init::defaultWindowPos;

	glutInit(&argc, argv);
	
	glutInitContextVersion(Core::Init::openGLMajorVersion, Core::Init::openGLMinorVersion);
	glutInitContextProfile(GLUT_CORE_PROFILE);
	
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_MULTISAMPLE | GLUT_RGBA | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowPosition((int)windowPos.x, (int)windowPos.y);
	glutInitWindowSize((int)windowSize.x, (int)windowSize.y);

	
	std::string asd(Core::Init::windowName.begin(), Core::Init::windowName.end());

	glutCreateWindow(asd.c_str());

	std::wcout << "GLUT initialized" << std::endl;

	glutIdleFunc(IdleCallback);
	glutCloseFunc(CloseCallback);
	glutDisplayFunc(DisplayCallback);
	glutReshapeFunc(ReshapeCallback);

	glutKeyboardFunc(KeyboardDownCallback);
	glutKeyboardUpFunc(KeyboardUpCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);

	glewExperimental = true;
	if (glewInit() == GLEW_OK)
	{
		std::wcout << "GLEW initialized successfully" << std::endl;
	}

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}

void Core::Init::InitGLUT::Run()
{
	std::wcout << "GLUT started running " << std::endl;
	glutMainLoop();
}

void Core::Init::InitGLUT::Close()
{
	std::wcout << "GLUT finished running" << std::endl;
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

void Core::Init::InitGLUT::SetSceneManager(Managers::SceneManager *&sceneManagerArg)
{
	sceneManager = sceneManagerArg;
	sceneManager->SetScreenProps((int)windowSize.x, (int)windowSize.y);
}

void Core::Init::InitGLUT::IdleCallback(void)
{
	glutPostRedisplay();
}

void Core::Init::InitGLUT::DisplayCallback(void)
{
	if (sceneManager)
	{
		sceneManager->notifyBeginFrame();
		sceneManager->notifyDisplayFrame();

		glutSwapBuffers();

		sceneManager->notifyEndFrame();
	}
}

void Core::Init::InitGLUT::ReshapeCallback(int width, int height)
{

	if (sceneManager)
	{
		sceneManager->notifyReshape(width, height, (int)windowSize.x, (int)windowSize.y);
		sceneManager->SetScreenProps(width, height);
		std::wcout << "GLUT reshaped" << std::endl;
	}
	windowSize = glm::vec2(width, height);
	glViewport(0, 0, width, height);

}

void Core::Init::InitGLUT::CloseCallback()
{
	Close();
}

void Core::Init::InitGLUT::KeyboardDownCallback(unsigned char key, int x, int y)
{
	sceneManager->KeyboardDownCallback(key, x, y);
}

void Core::Init::InitGLUT::KeyboardUpCallback(unsigned char key, int x, int y)
{
	sceneManager->KeyboardUpCallback(key, x, y);
}

void Core::Init::InitGLUT::MouseCallback(int button, int state, int x, int y)
{
	sceneManager->MouseCallback(button, state, x, y);
}

void Core::Init::InitGLUT::MotionCallback(int x, int y)
{
	sceneManager->MotionCallback(x, y);
}
