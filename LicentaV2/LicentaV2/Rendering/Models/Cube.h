#pragma once
#include "Model.h"
#include<time.h>
#include<stdarg.h>

namespace Rendering
{
	namespace Models
	{
		class Cube : public Model
		{
		public:
			Cube(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Cube();
			virtual void Create() override final;
		};
	}
}
