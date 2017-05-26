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
			Tetrahedron(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~Tetrahedron();
		};
	}
}

