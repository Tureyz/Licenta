#include "SceneManager.h"
#include "../Core/DeltaTime.h"

Managers::SceneManager::SceneManager()
{

	glEnable(GL_DEPTH_TEST);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//m_textRenderer = new TextRenderer();

	m_shaderManager = new ShaderManager();
	m_shaderManager->CreatePrograms();

	m_viewMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	//m_viewMatrix = glm::mat4(1.f);
	m_modelManager = new ModelManager();
	m_camera = new Rendering::Camera();
	//m_simulationManager = new BachelorSimulationManager(m_modelManager);
	m_simulationManager = new MastersSimulationManager(m_modelManager);

	//m_physicsManager = new PhysicsManager(m_modelManager->GetModelListPtr());

	m_modelManager->Init();
	m_simulationManager->Init();
	
	m_initTime = (float) clock();
	m_lastFixedTime = m_initTime;
	m_lastFrameTime = m_initTime;
	Core::DeltaTime::SetDt(0);
	//m_textRenderer->SetProgram(Managers::ShaderManager::GetTextShader());
	//m_textRenderer->Init();

}

Managers::SceneManager::~SceneManager()
{
	delete m_shaderManager;
	delete m_modelManager;
	delete m_simulationManager;
	//delete m_physicsManager;
}

void Managers::SceneManager::notifyBeginFrame()
{
// 	m_time = glutGet(GLUT_ELAPSED_TIME);
// 
// 	m_timeBase = m_time;
// 	m_deltaTime.UpdateTick();

	float crtTime = (float) clock();	

	if (crtTime - m_lastFixedTime >= Core::TIME_STEP_MS)
	{

		m_modelManager->FixedUpdate();
		m_simulationManager->FixedUpdate();
		//m_FPSCounter.FixedUpdate();

		int substeps = Core::TIME_STEP_MS / Core::PHYSICS_TIME_STEP_MS;
		for (int i = 1; i < substeps; ++i)
		{
			m_modelManager->FixedUpdate();
			m_simulationManager->FixedUpdate();
			//m_FPSCounter.FixedUpdate();

		}


		m_lastFixedTime = crtTime;
		//std::cout << "tick\n";
	}
	Core::DeltaTime::SetDt(crtTime - m_lastFrameTime);
	//std::cout << crtTime - m_lastFixedTime << " since last fixed, dt = " << Core::DeltaTime::GetDt() << std::endl;
	m_lastFrameTime = crtTime;
// 	m_physicsManager->FixedUpdate();
// 	m_physicsManager->SetCollisionPairs(m_simulationManager->GetCurrentCollisionPairsPtr());
// 	m_physicsManager->CollisionResponse();


	
}

void Managers::SceneManager::notifyDisplayFrame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(Core::BACKGROUND_COLOR.r, Core::BACKGROUND_COLOR.g, Core::BACKGROUND_COLOR.b, Core::BACKGROUND_COLOR.a);



	glUseProgram(Managers::ShaderManager::GetSceneShader());

	glm::vec3 light_position = glm::vec3(5.25f, 5.25f, 5.f);

	glUniform3f(glGetUniformLocation(Managers::ShaderManager::GetSceneShader(), "lightPosition"), light_position.x, light_position.y, light_position.z);
	glUniform3f(glGetUniformLocation(Managers::ShaderManager::GetSceneShader(), "eyePosition"), m_camera->GetEyeVector().x, m_camera->GetEyeVector().y, m_camera->GetEyeVector().z);
	glUniform1i(glGetUniformLocation(Managers::ShaderManager::GetSceneShader(), "shininess"), 6);
	glUniform1f(glGetUniformLocation(Managers::ShaderManager::GetSceneShader(), "kd"), 0.3);
	glUniform1f(glGetUniformLocation(Managers::ShaderManager::GetSceneShader(), "ks"), 0.3);
	glm::mat4 viewProjection = m_projectionMatrix * m_camera->GetViewMatrix();

	//m_FPSCounter.Draw();
	m_modelManager->Draw(viewProjection);

	m_simulationManager->Draw();
	m_simulationManager->Draw(viewProjection);


// 	/* Enable blending, necessary for our alpha texture */
// 	m_textRenderer->PutText("TEST", glm::vec2(200, 200));
// 	glUseProgram(Managers::ShaderManager::GetTextShader());
// 	glEnable(GL_BLEND);
// 	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 	m_textRenderer->Draw(m_width, m_height);
// 	glDisable(GL_BLEND);	


}

void Managers::SceneManager::notifyEndFrame()
{
	m_modelManager->Update();
	m_simulationManager->Update();
	//m_physicsManager->Update();
	//m_FPSCounter.Update();
	m_camera->Update();
}

void Managers::SceneManager::notifyReshape(int width, int height, int previousWidth, int previousHeight)
{
	float ar = (float)glutGet(GLUT_WINDOW_WIDTH) / (float)glutGet(GLUT_WINDOW_HEIGHT);
	float angle = 45.0f, near1 = 0.00001f, far1 = 2000.0f;

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
	//m_physicsManager->KeyPressed(key);
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
