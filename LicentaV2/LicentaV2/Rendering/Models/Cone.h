#pragma once
#include "Model.h"

namespace Rendering
{
	namespace Models
	{
		class Cone : public Model
		{
		public:
			Cone(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~Cone();
		};
	}
}
