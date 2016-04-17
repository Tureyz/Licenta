#pragma once
#include <vector>
#include "ShaderManager.h"
#include "../Rendering/IPhysicsObject.h"

namespace Managers
{
	enum physicsObjectType {OBJ_CUBE = 0, OBJ_SPHERE = 1, OBJ_TETRAHEDRON = 2, OBJ_RANDOM};
	class ModelManager
	{
	public:
		ModelManager();
		~ModelManager();

		void Init();
		void Draw();
		void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
		void Update();
		void DeleteModel(unsigned long id);
		void DeleteModelNDC(unsigned long id);
		const Rendering::IPhysicsObject* GetModel(unsigned long id) const;
		const Rendering::IPhysicsObject* GetModelNDC(unsigned long id) const;

		void SetModel(size_t id, Rendering::IPhysicsObject *gameObject);
		
		void SetBoundingBoxesVisibile(bool value);

		std::vector<Rendering::IPhysicsObject*> *GetModelListPtr() { return &m_physicsModelList; }
		std::vector<Rendering::VertexFormat> m_cubeVerts, m_sphereVerts, m_tetraVerts;
		std::vector<unsigned int> m_cubeIndices, m_tetraIndices, m_sphereIndices;
		GLuint m_cubeVao, m_cubeVbo, m_cubeIbo;
		GLuint m_tetraVao, m_tetraVbo, m_tetraIbo;
		GLuint m_sphereVao, m_sphereVbo, m_sphereIbo;

	private:
		void CreateBufferObjects();

		std::vector<Rendering::IPhysicsObject*> m_physicsModelList;
		std::vector<Rendering::IPhysicsObject*> m_physicsModelListNDC;
	};
}