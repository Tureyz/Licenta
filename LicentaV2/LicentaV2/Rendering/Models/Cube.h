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
			Cube(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~Cube();
		};
	}
}
