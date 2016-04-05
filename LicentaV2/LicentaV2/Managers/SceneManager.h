#pragma once
#include "../Benchmark/FPSCounter.h"
#include "ShaderManager.h"
#include "ModelManager.h"
#include "../Core/Init/ListenerInterface.h"
#include "../Rendering/Camera.h"

namespace Managers
{
	class SceneManager : public Core::IListener
	{
	public:
		SceneManager();
		~SceneManager();

		virtual void notifyBeginFrame();
		virtual void notifyDisplayFrame();
		virtual void notifyEndFrame();
		virtual void notifyReshape(int width, int height, int previous_width, int previous_height);

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
		glm::mat4 m_projectionMatrix;
		glm::mat4 m_viewMatrix;
		Benchmark::FPSCounter m_FPSCounter;
	};
}