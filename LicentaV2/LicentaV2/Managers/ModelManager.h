#pragma once
#include <vector>
#include "ShaderManager.h"
#include "../Rendering/IPhysicsObject.h"

namespace Managers
{
	class ModelManager
	{
	public:
		ModelManager();
		~ModelManager();

		void Init();
		void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
		void FixedUpdate();
		void Update();
		void DeleteAllModels();
		void DeleteModel(unsigned long id);
		const Rendering::IPhysicsObject* GetModel(unsigned long id) const;

		void RegisterObject(size_t id, Rendering::IPhysicsObject *gameObject);
		
		void SetBoundingBoxesVisibile(bool value);

		std::vector<Rendering::IPhysicsObject*> *GetModelListPtr() { return &m_objectList; }

		std::vector<Rendering::VertexFormat> m_cubeVerts, m_sphereVerts, m_tetraVerts, m_cylinderVerts, m_coneVerts;
		//care
		std::vector<GLuint> m_cubeIndices, m_tetraIndices, m_sphereIndices, m_cylinderIndices, m_coneIndices, m_lineCubeIndices;
		GLuint m_cubeVao, m_cubeVbo, m_cubeIbo;
		GLuint m_tetraVao, m_tetraVbo, m_tetraIbo;
		GLuint m_sphereVao, m_sphereVbo, m_sphereIbo;
		GLuint m_cylinderVao, m_cylinderVbo, m_cylinderIbo;
		GLuint m_coneVao, m_coneVbo, m_coneIbo;

		float GetDt() const { return m_dt; }
		void SetDt(float val) { m_dt = val; }
	private:
		void CreateBufferObjects();
		void CreateCubeProps();
		void CreateTetrahedronProps();
		void CreateConeProps();
		void CreateCylinderProps();
		void CreateSphereProps();

		std::vector<Rendering::IPhysicsObject*> m_objectList;

		float m_dt;
	};
}