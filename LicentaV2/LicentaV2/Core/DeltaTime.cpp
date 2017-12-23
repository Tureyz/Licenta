#include "DeltaTime.h"
#include "../Dependencies/freeglut/freeglut.h"


float Core::DeltaTime::GetDt()
{
	return m_dt;
}

void Core::DeltaTime::SetDt(float value)
{
	m_dt = value;
}
