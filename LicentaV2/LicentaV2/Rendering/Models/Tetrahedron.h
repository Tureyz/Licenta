#pragma once
#include "Model.h"
#include<time.h>
#include<stdarg.h>

namespace Rendering
{
	namespace Models
	{
		class Tetrahedron : public Model
		{
		public:
			Tetrahedron(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Tetrahedron();
			virtual void Create() override final;
		};
	}
}

