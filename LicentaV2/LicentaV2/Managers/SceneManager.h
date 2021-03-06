#pragma once
//#include "../Benchmark/FPSCounter.h"
#include "..\Core\Utils.hpp"
#include "ShaderManager.h"
#include "ModelManager.h"
#include "BachelorSimulationManager.h"
#include "MastersSimulationManager.h"
#include "PhysicsManager.h"
//#include "../Rendering/TextRenderer.h"

//#include "../Core/DeltaTime.h"
#include "../Rendering/Camera.h"

namespace Managers
{
	class SceneManager
	{
	public:
		SceneManager();
		~SceneManager();

		virtual void notifyBeginFrame();
		virtual void notifyDisplayFrame();
		virtual void notifyEndFrame();
		virtual void notifyReshape(int width, int height, int previousWidth, int previousHeight);

		virtual void KeyboardDownCallback(unsigned char key, int x, int y);
		virtual void KeyboardUpCallback(unsigned char key, int x, int y);
		virtual void MouseCallback(int button, int state, int x, int y);
		virtual void MotionCallback(int x, int y);

		Managers::ModelManager* GetModelManager();

		void SetScreenProps(int width, int height);

	private:
		int m_width;
		int m_height;
		Rendering::Camera *m_camera;
		//Rendering::TextRenderer *m_textRenderer;

		Managers::ShaderManager *m_shaderManager;
		Managers::ModelManager *m_modelManager;
		//Managers::BachelorSimulationManager *m_simulationManager;
		Managers::MastersSimulationManager *m_simulationManager;
		Managers::PhysicsManager *m_physicsManager;

		glm::mat4 m_projectionMatrix;
		glm::mat4 m_viewMatrix;
		//Benchmark::FPSCounter m_FPSCounter;

// 		Core::DeltaTime m_deltaTime;
// 		int m_timeBase;
// 		int m_time;

		float m_initTime;
		float m_lastFixedTime; // last fixed update
		float m_lastFrameTime;
		float m_dt; // difference between this rendered frame and last fixed update
	};
}