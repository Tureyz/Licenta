#include "SceneManager.h"

Managers::SceneManager::SceneManager()
{
	glEnable(GL_DEPTH_TEST);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	m_shaderManager = new ShaderManager();
	m_shaderManager->CreateProgram("colorShader", "Shaders\\VertexShader.glsl", "Shaders\\FragmentShader.glsl");

	m_viewMatrix = glm::mat4(
		1.0f, 0.0f,  0.0f, 0.0f,
		0.0f, 1.0f,  0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	//m_viewMatrix = glm::mat4(1.f);
	m_modelManager = new ModelManager();
	m_camera = new Rendering::Camera();
	

}

Managers::SceneManager::~SceneManager()
{
	delete m_shaderManager;
	delete m_modelManager;
}

void Managers::SceneManager::notifyBeginFrame()
{
	m_modelManager->Update();
	m_FPSCounter.Update();
}

void Managers::SceneManager::notifyDisplayFrame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.2, 0.2, 0.2, 1.0);

	glUseProgram(Managers::ShaderManager::GetShader("colorShader"));
	m_FPSCounter.Draw();
	m_modelManager->Draw();
	m_modelManager->Draw(m_projectionMatrix, m_camera->GetViewMatrix());
}

void Managers::SceneManager::notifyEndFrame()
{
}

void Managers::SceneManager::notifyReshape(int width, int height, int previous_width, int previous_height)
{
	float ar = (float)glutGet(GLUT_WINDOW_WIDTH) / (float)glutGet(GLUT_WINDOW_HEIGHT);
	float angle = 45.0f, near1 = 0.1f, far1 = 2000.0f;

	m_projectionMatrix[0][0] = 1.0f / (ar * tan(angle / 2.0f));
	m_projectionMatrix[1][1] = 1.0f / tan(angle / 2.0f);
	m_projectionMatrix[2][2] = (-near1 - far1) / (near1 - far1);
	m_projectionMatrix[2][3] = 1.0f;
	m_projectionMatrix[3][2] = 2.0f * near1 * far1 / (near1 - far1);
}

void Managers::SceneManager::KeyboardCallback(unsigned char key, int x, int y)
{
	m_camera->KeyPressed(key);
}

void Managers::SceneManager::MouseCallback(int button, int state, int x, int y)
{
	m_camera->MousePressed(button, state, x, y);
}

void Managers::SceneManager::MotionCallback(int x, int y)
{
	m_camera->MouseMove(x, y, m_width, m_height);
}

Managers::ModelManager * Managers::SceneManager::GetModelManager()
{
	return m_modelManager;
}

void Managers::SceneManager::SetScreenProps(int width, int height)
{
	m_width = width;
	m_height = height;
}
