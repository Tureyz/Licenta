#pragma once
#include <vector>
#include "../../Managers/ModelManager.h"
#include "../IPhysicsObject.h"
#include "../../Collision/DataStructures/BoundingBox.h"
#include "../../Managers/SimulationManager.h"

namespace Rendering
{
	namespace Models 
	{
		
		class Model : public IPhysicsObject
		{
		public:
			Model(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager);
			virtual ~Model();
			// methods from interface
			virtual void Create() override;
			virtual void Draw() override;
			virtual void Draw(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) override;
			virtual void Update() override;
			virtual void SetProgram(GLuint shaderName) override;
			virtual void Destroy() override;

			virtual GLuint GetVao() const override;
			virtual const std::vector<GLuint>& GetVbos() const override;

			virtual void TranslateAbsolute(const glm::vec3 &pos) override;
			virtual void RotateAbsolute(const glm::vec3 &axis, const float angles) override;
			virtual void ScaleAbsolute(const glm::vec3 &scales) override;

			virtual void TranslateRelative(const glm::vec3 &pos) override;
			virtual void RotateRelative(const glm::vec3 &axis, const float angles) override;
			virtual void ScaleRelative(const glm::vec3 &scales) override;

			virtual void DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) override;

			virtual void UpdateVertices(glm::mat4 mat) override;

			virtual void SetBoundingBoxVisible(bool value);

			Collision::DataStructures::BoundingBox * GetBoundingBox() const { return m_boundingBox; }
			void SetBoundingBox(Collision::DataStructures::BoundingBox * val) { m_boundingBox = val; }
			Managers::ModelManager * GetModelManager() const { return m_modelManager; }
			void SetModelManager(Managers::ModelManager * val) { m_modelManager = val; }
			GLuint m_program;
			Managers::SimulationManager * GetSimulationManager() const { return m_simulationManager; }
			void SetSimulationManager(Managers::SimulationManager * val) { m_simulationManager = val; }

			virtual void ObjectMoved() override;

		protected:
			GLuint m_vao;
			Managers::ModelManager *m_modelManager;
			Managers::SimulationManager *m_simulationManager;
			std::vector<GLuint> m_vbos;
			Collision::DataStructures::BoundingBox *m_boundingBox;
		};
	}
}