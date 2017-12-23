#pragma once
#include <vector>
#include "../../Managers/ISimulationManager.h"

namespace Rendering
{
	namespace Models
	{
		class Model : public IPhysicsObject
		{
		public:
			Model(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager);
			Model(const Model &other);
			virtual ~Model();

			virtual void Create() override;
			virtual void Create(const glm::mat4 &mvp);
			virtual void Draw() override;
			virtual void Draw(const glm::mat4& viewProjection) override final;

			virtual void Destroy() override;
			virtual void SetBoundingBoxVisible(bool value);
			virtual void ObjectMoved() override;

			Collision::DataStructures::BoundingBox  GetBoundingBox() const { return m_boundingBox; }
			void SetBoundingBox(Collision::DataStructures::BoundingBox  val) { m_boundingBox = val; }

			Managers::ModelManager * GetModelManager() const { return m_modelManager; }
			void SetModelManager(Managers::ModelManager * val) { m_modelManager = val; }

			Managers::ISimulationManager * GetSimulationManager() const { return m_simulationManager; }
			void SetSimulationManager(Managers::ISimulationManager * val) { m_simulationManager = val; }

		protected:

			Managers::ModelManager *m_modelManager;
			Managers::ISimulationManager *m_simulationManager;
		};
	}
}