#pragma once
#include <vector>
#include "ShaderManager.h"
#include "../Collision/SpatialGrid.h"
#include "../Collision/BVH.h"

namespace Managers
{
	enum physicsObjectType {OBJ_CUBE = 0, OBJ_SPHERE = 1, OBJ_TETRAHEDRON = 2, OBJ_RANDOM};
	class ModelManager
	{
	public:
		ModelManager();
		~ModelManager();

		void Draw();
		void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
		void Update();
		void DeleteModel(unsigned long id);
		void DeleteModelNDC(unsigned long id);
		const IPhysicsObject* GetModel(unsigned long id) const;
		const IPhysicsObject* GetModelNDC(unsigned long id) const;

		void SetModel(unsigned long id, IPhysicsObject *gameObject);

		void SpawnObjectAt(const glm::vec3 &position, const physicsObjectType objectType, const glm::vec4 &color);
		void SpawnManyAround(const glm::vec3 &position, const float radius, const int numberOfObjects, Managers::physicsObjectType typeOfObjects);

		void TestCollision();
		void SetBoundingBoxesVisibile(bool value);

		void Init();

		std::vector<VertexFormat> m_cubeVerts, m_sphereVerts, m_tetraVerts;
		std::vector<unsigned int> m_cubeIndices, m_tetraIndices, m_sphereIndices;
		GLuint m_cubeVao, m_cubeVbo, m_cubeIbo;
		GLuint m_tetraVao, m_tetraVbo, m_tetraIbo;
		GLuint m_sphereVao, m_sphereVbo, m_sphereIbo;

	private:
		void CreateBufferObjects();

		std::vector<IPhysicsObject*> m_physicsModelList;
		std::vector<IPhysicsObject*> m_physicsModelListNDC;
		glm::vec3 m_objectCounters;
		bool m_inited;
		bool asdd;
		Collision::BVH *m_test;
		unsigned long m_objectIDCounter;
		int m_time;
		int m_timeBase;

	};
}