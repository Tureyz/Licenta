#pragma once
#include "Core\Init\InitGLUT.h"
#include "Managers\SceneManager.h"
#include "Rendering\Models\Cube.h"

using namespace Core;

int main(int argc, char **argv)
{
	std::srand((unsigned int) time(NULL));
	WindowProperties window(std::string("Test Licenta"), true, glm::vec2(400, 200), glm::vec2(1024, 768));

	ContextProperties context(4, 5, true);
	FrameBufferProperties frameBufferProperties(true, true, true, true);
	Init::InitGLUT::InitGLUTFunc(window, context, frameBufferProperties, argc, argv);

	Managers::SceneManager *scene = new Managers::SceneManager();
	Init::InitGLUT::SetListener(scene);


// 	Rendering::Models::Cube *cube = new Rendering::Models::Cube();
// 	cube->SetProgram(Managers::ShaderManager::GetShader("colorShader"));
// 	cube->Create();
// 	cube->RotateRelative(glm::vec3(0.0f, 1.0f, 0.0f), 120);
// 	scene->GetModelManager()->SetModel(std::string("cube"), cube);
// 
//  	Rendering::Models::Cube *cube2 = new Rendering::Models::Cube();
//  	cube2->SetProgram(Managers::ShaderManager::GetShader("colorShader"));
//  	cube2->Create();
// 	cube2->ScaleRelative(glm::vec3(0.5f));
// 	cube2->TranslateRelative(glm::vec3(0.0, 2, 0.0));
// 
// 	scene->GetModelManager()->SetModel(std::string("cube2"), cube2);


	Init::InitGLUT::Run();


	delete scene;
	return 0;
}