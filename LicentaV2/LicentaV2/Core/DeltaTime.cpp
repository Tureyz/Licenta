#include "DeltaTime.h"
#include "../Dependencies/freeglut/freeglut.h"

Core::DeltaTime::DeltaTime()
{
	dt = 0;
	t1 = glutGet(GLUT_ELAPSED_TIME);
}

void Core::DeltaTime::UpdateTick()
{
	t1 = glutGet(GLUT_ELAPSED_TIME);	
}

float Core::DeltaTime::GetDeltaTime()
{
	int t2 = glutGet(GLUT_ELAPSED_TIME);

	return ((float)(t2 - t1)) / 1000.f;
}
