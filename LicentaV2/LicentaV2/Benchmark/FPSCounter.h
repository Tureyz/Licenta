#pragma once

namespace Benchmark
{
	class FPSCounter
	{
	private:
		int m_frames;
		int m_time;
		int m_timeBase;
		float m_fps;
	public:
		void Update();
		void Draw();
	};
}