#pragma once
#include "Model.h"

namespace Rendering
{
	namespace Models
	{
		class Cylinder : public Model
		{
		public:
			Cylinder(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Cylinder();

			virtual void Create() override final;
		};
	}
}