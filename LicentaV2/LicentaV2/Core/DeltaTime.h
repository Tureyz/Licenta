#pragma once

namespace Core
{
	class DeltaTime
	{
	public:
		DeltaTime();
		void UpdateTick();
		float GetDeltaTime();
	private:
		int t1;
		float dt;
	};
}