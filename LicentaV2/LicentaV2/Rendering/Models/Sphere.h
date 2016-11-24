#pragma once
#include "Model.h"
#include<time.h>
#include<stdarg.h>

namespace Rendering
{
	namespace Models
	{
		class Sphere : public Model
		{
		public:
			Sphere(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Sphere();

			virtual void Create() override final;			
			virtual void Create(const glm::mat4 &mvp) override final;
		};
	}
}
