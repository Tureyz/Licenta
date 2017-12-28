#include "Camera.h"
#include "../Dependencies/glm/gtx/rotate_vector.hpp"
#include "../Dependencies/freeglut/freeglut_std.h"
#include "../Core/DeltaTime.h"



Rendering::Camera::Camera()
{
	m_viewMatrix = glm::mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, -1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	SetEyeVector(glm::vec3(5.25f, 5.25f, -4.f));
	m_keyPitch = m_keyRoll = m_keyYaw = 0.f;
	m_cameraQuat = glm::quat(0.f, 0.f, 0.f, 0.f);
	m_isMousePressed = false;
	m_leftPressed = m_forwardPressed = m_backwardPressed = m_rightPressed = false;
	UpdateView();
}

Rendering::Camera::~Camera()
{
}

void Rendering::Camera::UpdateView()
{
	m_cameraQuat = glm::normalize(glm::quat(-glm::vec3(m_keyPitch, m_keyYaw, m_keyRoll)) * m_cameraQuat);
	m_keyPitch = m_keyYaw = m_keyRoll = 0;
	m_viewMatrix = glm::mat4_cast(m_cameraQuat) * glm::translate(glm::mat4(1.0f), -GetEyeVector());
}

glm::mat4 Rendering::Camera::GetViewMatrix() const
{
	return m_viewMatrix;
}

void Rendering::Camera::KeyPressed(const unsigned char key)
{
	switch (key)
	{
	case 'a':
		m_leftPressed = true;
		break;
	case 'd':
		m_rightPressed = true;
		break;
	case 's':
		m_backwardPressed = true;
		break;
	case 'w':
		m_forwardPressed = true;
		break;
	default:
		break;
	}
}

void Rendering::Camera::KeyReleased(const unsigned char key)
{
	switch (key)
	{
	case 'a':
		m_leftPressed = false;
		break;
	case 'd':
		m_rightPressed = false;
		break;
	case 's':
		m_backwardPressed = false;
		break;
	case 'w':
		m_forwardPressed = false;
		break;
	default:
		break;
	}
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
		m_mousePosition.x = (float)x;
		m_mousePosition.y = (float)y;
	}
}

void Rendering::Camera::Update()
{
	float distancePerStep = 0.025f;

	float dx = ((m_leftPressed ? -distancePerStep : 0) + (m_rightPressed ? distancePerStep : 0));
	float dz = ((m_forwardPressed ? -distancePerStep : 0) + (m_backwardPressed ? distancePerStep : 0));

	glm::mat4 mat = GetViewMatrix();

	glm::vec3 forward(mat[0][2], mat[1][2], mat[2][2]);
	glm::vec3 strafe(mat[0][0], mat[1][0], mat[2][0]);

	const float speed = 0.12f;
	SetEyeVector(GetEyeVector() + (-dz * forward + dx * strafe) * speed * Core::DeltaTime::GetDt());

	UpdateView();
}
