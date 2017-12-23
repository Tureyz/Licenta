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
		void Draw(const glm::mat4& viewProjection);
		void FixedUpdate();
		void Update();
		void DeleteAllModels();
		void DeleteModel(std::size_t id);
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

		Rendering::VisualBody CreateBasicVisualBody(enum Simulation::PhysicsObjectType type);
		Rendering::VisualBody CreateMeshVisualBody(const int rows, const int cols);
		
		void CreateMeshBufferObjects(const int rows, const int cols, GLuint &vao, GLuint &vbo, GLuint &ibo, std::vector<Rendering::VertexFormat> &verts, std::vector<GLuint> &indices);
		void CreateCubeBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo);
		void CreateTetrahedronBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo);
		void CreateConeBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo);
		void CreateCylinderBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo);
		void CreateSphereBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo);
	private:
		void CreateProps();
		void CreateCubeProps();
		void CreateTetrahedronProps();
		void CreateConeProps();
		void CreateCylinderProps();
		void CreateSphereProps();
		void SubdivideTriangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, std::vector<glm::vec3> &verts, std::vector<unsigned int> &indices);
		std::pair<std::vector<Rendering::VertexFormat>, std::vector<GLuint>> CreateMeshProps(int rows, int cols);


		std::vector<Rendering::IPhysicsObject*> m_objectList;

		float m_dt;		
	};
}