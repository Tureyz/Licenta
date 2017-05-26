#pragma once
#include "Model.h"

namespace Rendering
{
	namespace Models
	{
		class Cylinder : public Model
		{
		public:
			Cylinder(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~Cylinder();
		};
	}
}