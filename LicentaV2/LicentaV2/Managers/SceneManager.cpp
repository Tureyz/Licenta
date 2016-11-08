#include "SceneManager.h"
#include "..\Core\Utils.hpp"

Managers::SceneManager::SceneManager()
{
	m_time = m_timeBase = 0;

	glEnable(GL_DEPTH_TEST);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	m_shaderManager = new ShaderManager();
	m_shaderManager->CreateProgram(L"Shaders\\VertexShader.glsl", L"Shaders\\FragmentShader.glsl");

	m_viewMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	//m_viewMatrix = glm::mat4(1.f);
	m_modelManager = new ModelManager();
	m_camera = new Rendering::Camera();
	m_simulationManager = new SimulationManager(m_modelManager);
	m_physicsManager = new PhysicsManager(m_modelManager->GetModelListPtr());

	m_modelManager->Init();
	m_simulationManager->Init();
	m_deltaTime.UpdateTick();
}

Managers::SceneManager::~SceneManager()
{
	delete m_shaderManager;
	delete m_modelManager;
	delete m_simulationManager;
	delete m_physicsManager;
}

void Managers::SceneManager::notifyBeginFrame()
{
	m_time = glutGet(GLUT_ELAPSED_TIME);

	m_timeBase = m_time;
	m_deltaTime.UpdateTick();

	m_modelManager->FixedUpdate();
	m_simulationManager->FixedUpdate();
	m_FPSCounter.FixedUpdate();
	m_physicsManager->FixedUpdate();
	m_physicsManager->SetCollisionPairs(m_simulationManager->GetCurrentCollisionPairsPtr());
	m_physicsManager->CollisionResponse();


	m_modelManager->Update();
	m_simulationManager->Update();
	m_physicsManager->Update();
	m_FPSCounter.Update();
	m_camera->Update();
}

void Managers::SceneManager::notifyDisplayFrame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(Core::BACKGROUND_COLOR.r, Core::BACKGROUND_COLOR.g, Core::BACKGROUND_COLOR.b, Core::BACKGROUND_COLOR.a);

	m_FPSCounter.Draw();
	glUseProgram(Managers::ShaderManager::GetShader());
	m_modelManager->Draw(m_projectionMatrix, m_camera->GetViewMatrix());

	m_simulationManager->Draw();
	m_simulationManager->Draw(m_projectionMatrix, m_camera->GetViewMatrix());
}

void Managers::SceneManager::notifyEndFrame()
{
}

void Managers::SceneManager::notifyReshape(int width, int height, int previousWidth, int previousHeight)
{
	float ar = (float)glutGet(GLUT_WINDOW_WIDTH) / (float)glutGet(GLUT_WINDOW_HEIGHT);
	float angle = 45.0f, near1 = 0.1f, far1 = 2000.0f;

	m_projectionMatrix[0][0] = 1.0f / (ar * tan(angle / 2.0f));
	m_projectionMatrix[1][1] = 1.0f / tan(angle / 2.0f);
	m_projectionMatrix[2][2] = (-near1 - far1) / (near1 - far1);
	m_projectionMatrix[2][3] = 1.0f;
	m_projectionMatrix[3][2] = 2.0f * near1 * far1 / (near1 - far1);
}

void Managers::SceneManager::KeyboardDownCallback(unsigned char key, int x, int y)
{
	m_camera->KeyPressed(key);
	m_simulationManager->KeyPressed(key);
}

void Managers::SceneManager::KeyboardUpCallback(unsigned char key, int x, int y)
{
	m_camera->KeyReleased(key);
	m_simulationManager->KeyReleased(key);
}

void Managers::SceneManager::MouseCallback(int button, int state, int x, int y)
{
	m_camera->MousePressed(button, state, x, y);
	m_simulationManager->MousePressed(button, state, x, y);
}

void Managers::SceneManager::MotionCallback(int x, int y)
{
	m_camera->MouseMove(x, y, m_width, m_height);
	m_simulationManager->MouseMove(x, y, m_width, m_height);
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
