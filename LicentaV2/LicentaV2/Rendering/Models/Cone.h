#pragma once
#include "Model.h"

namespace Rendering
{
	namespace Models
	{
		class Cone : public Model
		{
		public:
			Cone(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Cone();
			virtual void Create() override final;
		};
	}
}
