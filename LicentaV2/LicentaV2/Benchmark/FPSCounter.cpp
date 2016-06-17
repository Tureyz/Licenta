#include "FPSCounter.h"
#include "../Core/Utils.hpp"
#include "../Dependencies/freeglut/freeglut.h"

void Benchmark::FPSCounter::Update()
{
	m_frames++;

	m_time = glutGet(GLUT_ELAPSED_TIME);

	int delta = m_time - m_timeBase;

	if (delta > 1000)
	{
		m_fps = m_frames / (delta / 1000.f);
		m_timeBase = m_time;
		m_frames = 0;
		//std::wcout << "FPS: " << m_fps << std::endl;
	}

}

void Benchmark::FPSCounter::FixedUpdate()
{

}

void Benchmark::FPSCounter::Draw()
{
	// Doesn't work, used a cout ^^^^
	Core::Utils::printToScreen(glm::vec2(0, 0), std::to_wstring(m_fps));
}
