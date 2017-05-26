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
			Sphere(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~Sphere();
		};
	}
}
