#pragma once

namespace Core
{
	static float m_dt;

	class DeltaTime
	{
	public:		
		static float GetDt();
		static void SetDt(float ovalue);
	private:		
	};
}