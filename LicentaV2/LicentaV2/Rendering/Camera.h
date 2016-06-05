#pragma once
#include "../Dependencies/glm/glm.hpp"
#include "../Dependencies/glm/gtx/quaternion.hpp"
namespace Rendering
{
	class Camera
	{
	public:
		Camera();
		~Camera();
		void UpdateView();
		glm::mat4 GetViewMatrix() const;
		void KeyPressed(const unsigned char key);
		void MouseMove(int x, int y, int width, int height);
		void MousePressed(int button, int state, int x, int y);

	private:

		glm::mat4 m_viewMatrix;
		glm::vec3 m_eyeVector;
		bool m_isMousePressed;
		glm::vec2 m_mousePosition;
		float m_keyPitch;
		float m_keyYaw;
		float m_keyRoll;
		glm::quat m_cameraQuat;
	};
}