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
	m_eyeVector = glm::vec3(0.f);
	m_keyPitch = m_keyRoll = m_keyYaw = 0.f;
	m_cameraQuat = glm::quat(1.f, 1.f, 1.f, 1.f);
	m_isMousePressed = false;
}

Rendering::Camera::~Camera()
{
}

void Rendering::Camera::UpdateView()
{
	//temporary frame quaternion from pitch,yaw,roll 
	//here roll is not used
	glm::quat key_quat = glm::quat(-glm::vec3(m_keyPitch, m_keyYaw, m_keyRoll));
	//reset values
	m_keyPitch = m_keyYaw = m_keyRoll = 0;

	//order matters,update camera_quat
	m_cameraQuat = key_quat * m_cameraQuat;
	m_cameraQuat = glm::normalize(m_cameraQuat);
	glm::mat4 rotate = glm::mat4_cast(m_cameraQuat);

	glm::mat4 translate = glm::mat4(1.0f);
	translate = glm::translate(translate, -m_eyeVector);

	m_viewMatrix = rotate * translate;
}

glm::mat4 Rendering::Camera::GetViewMatrix() const
{
	return m_viewMatrix;
}

void Rendering::Camera::KeyPressed(const unsigned char key)
{
	float dx = 0; //how much we strafe on x
	float dz = 0; //how much we walk on z
	switch (key)
	{
	case 'w':
	{
		dz = -2;
		break;
	}

	case 's':
	{
		dz = 2;
		break;
	}
	case 'a':
	{
		dx = -2;
		break;
	}

	case 'd':
	{
		dx = 2;
		break;
	}
	default:
		break;
	}

	//get current view matrix
	glm::mat4 mat = GetViewMatrix();
	//row major
	glm::vec3 forward(mat[0][2], mat[1][2], mat[2][2]);
	glm::vec3 strafe(mat[0][0], mat[1][0], mat[2][0]);

	const float speed = 0.12f;
	m_eyeVector += (-dz * forward + dx * strafe) * speed;

	//update the view matrix
	UpdateView();
}

void Rendering::Camera::MouseMove(int x, int y, int width, int height)
{
	if (m_isMousePressed == false)
		return;
	//always compute delta
	//mousePosition is the last mouse position
	glm::vec2 mouse_delta = glm::vec2(x, y) - m_mousePosition;

	//notice that we reduce the sensitvity
	const float mouseX_Sensitivity = 0.0020f;
	const float mouseY_Sensitivity = 0.0020f;

	m_keyYaw = mouseX_Sensitivity * mouse_delta.x;
	m_keyPitch = mouseY_Sensitivity * mouse_delta.y;

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
