#pragma once
#include "Model.h"

namespace Rendering
{
	namespace Models
	{
		class MeshObject : public Model
		{
		public:
			MeshObject(int rows, int cols, Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			~MeshObject();
			virtual void Create() override final;
			virtual void Create(const glm::mat4 &mvp) override final;
		private:
			int m_rows, m_cols;

		};
	}
}