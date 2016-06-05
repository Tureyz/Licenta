#pragma once
#include "../Benchmark/FPSCounter.h"
#include "ShaderManager.h"
#include "ModelManager.h"
#include "SimulationManager.h"

#include "../Core/DeltaTime.h"
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

		virtual void KeyboardCallback(unsigned char key, int x, int y);
		virtual void MouseCallback(int button, int state, int x, int y);
		virtual void MotionCallback(int x, int y);

		Managers::ModelManager* GetModelManager();

		void SetScreenProps(int width, int height);

	private:
		int m_width;
		int m_height;
		Rendering::Camera *m_camera;
		Managers::ShaderManager *m_shaderManager;
		Managers::ModelManager *m_modelManager;
		Managers::SimulationManager *m_simulationManager;
		glm::mat4 m_projectionMatrix;
		glm::mat4 m_viewMatrix;
		Benchmark::FPSCounter m_FPSCounter;

		Core::DeltaTime m_deltaTime;
		int m_timeBase;
		int m_time;
	};
}