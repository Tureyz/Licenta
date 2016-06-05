#include "Camera.h"
#include "../Dependencies/glm/gtx/rotate_vector.hpp"
#include "../Dependencies/freeglut/freeglut_std.h"



Rendering::Camera::Camera()
{
	m_viewMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	m_eyeVector = glm::vec3(0.f, 0.f, -20.f);
	m_keyPitch = m_keyRoll = m_keyYaw = 0.f;
	m_cameraQuat = glm::quat(0.f, 0.f, 0.f, 0.f);
	m_isMousePressed = false;
	UpdateView();
}

Rendering::Camera::~Camera()
{
}

void Rendering::Camera::UpdateView()
{
	m_cameraQuat = glm::normalize(glm::quat(-glm::vec3(m_keyPitch, m_keyYaw, m_keyRoll)) * m_cameraQuat);
	m_keyPitch = m_keyYaw = m_keyRoll = 0;
	m_viewMatrix = glm::mat4_cast(m_cameraQuat) * glm::translate(glm::mat4(1.0f), -m_eyeVector);
}

glm::mat4 Rendering::Camera::GetViewMatrix() const
{
	return m_viewMatrix;
}

void Rendering::Camera::KeyPressed(const unsigned char key)
{
	float distancePerStep = 2;
	float dx = key == 'a' ? -distancePerStep : key == 'd' ? distancePerStep : 0;
	float dz = key == 'w' ? -distancePerStep : key == 's' ? distancePerStep : 0;
	
	glm::mat4 mat = GetViewMatrix();
	
	glm::vec3 forward(mat[0][2], mat[1][2], mat[2][2]);
	glm::vec3 strafe(mat[0][0], mat[1][0], mat[2][0]);

	const float speed = 0.12f;
	m_eyeVector += (-dz * forward + dx * strafe) * speed;
	
	UpdateView();
}

void Rendering::Camera::MouseMove(int x, int y, int width, int height)
{
	if (m_isMousePressed == false)
		return;
	glm::vec2 mouseDelta = glm::vec2(x, y) - m_mousePosition;

	const float mouseXSensitivity = 0.0020f;
	const float mouseYSensitivity = 0.0020f;

	m_keyYaw = mouseXSensitivity * mouseDelta.x;
	m_keyPitch = mouseYSensitivity * mouseDelta.y;

	m_mousePosition = glm::vec2(x, y);
	UpdateView();
}

void Rendering::Camera::MousePressed(int button, int state, int x, int y)
{
	if (state == GLUT_UP)
	{
		m_isMousePressed = false;
	}
	if (state == GLUT_DOWN)
	{
		m_isMousePressed = true;
		m_mousePosition.x = (float) x;
		m_mousePosition.y = (float) y;
	}
}
