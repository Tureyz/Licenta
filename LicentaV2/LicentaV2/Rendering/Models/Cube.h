#pragma once
#include "Model.h"
#include<time.h>
#include<stdarg.h>

namespace Rendering
{
	namespace Models
	{
		class BoundingBox;
		class Cube : public Model
		{
		public:
			Cube(const glm::vec4 &color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			~Cube();

			virtual void Create() override final;
			virtual void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override final;
			virtual void Update() override final;

			virtual void DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) override;

		private:
			time_t m_timer;
		};
	}
}
