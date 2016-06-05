#pragma once
#include "Core\InitGLUT.h"
#include "Managers\SceneManager.h"
#include "Rendering\Models\Cube.h"

using namespace Core;

int main(int argc, char **argv)
{
	std::srand((unsigned int) time(NULL));
	Init::InitGLUT::InitGLUTFunc(argc, argv);

	Managers::SceneManager *scene = new Managers::SceneManager();
	Init::InitGLUT::SetSceneManager(scene);

	Init::InitGLUT::Run();

	delete scene;
	return 0;
}