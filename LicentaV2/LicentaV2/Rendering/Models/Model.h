#pragma once
#include <vector>
#include "../IPhysicsObject.h"
#include "../../Collision/DataStructures/BoundingBox.h"
#include "../../Managers/SimulationManager.h"
#include "../../Managers/ModelManager.h"

namespace Rendering
{
	namespace Models 
	{		
		class Model : public IPhysicsObject
		{
		public:
			Model(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			Model(const Model &other);
			virtual ~Model();

			virtual void Create() override;
			virtual void Draw() override;
			virtual void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override final;
			virtual void DrawBB(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override final;

			virtual void Destroy() override;
			virtual void SetBoundingBoxVisible(bool value);
			virtual void ObjectMoved() override;

			Collision::DataStructures::BoundingBox * GetBoundingBox() const { return m_boundingBox; }
			void SetBoundingBox(Collision::DataStructures::BoundingBox * val) { m_boundingBox = val; }

			Managers::ModelManager * GetModelManager() const { return m_modelManager; }
			void SetModelManager(Managers::ModelManager * val) { m_modelManager = val; }

			Managers::SimulationManager * GetSimulationManager() const { return m_simulationManager; }
			void SetSimulationManager(Managers::SimulationManager * val) { m_simulationManager = val; }

		protected:

			Collision::DataStructures::BoundingBox *m_boundingBox;

			Managers::ModelManager *m_modelManager;
			Managers::SimulationManager *m_simulationManager;			
		};
	}
}