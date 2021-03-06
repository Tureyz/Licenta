#pragma once
#include <vector>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "..\Core\Utils.hpp"
#include "..\Rendering\SceneObject.h"


using namespace Rendering;

namespace Collision
{
	class ICollisionMethod
	{
	public:

		virtual ~ICollisionMethod() = 0;

		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> TestCollision() final;
		virtual void Update() final;

		virtual void DrawDebug(const glm::mat4& viewProjection) = 0;

		virtual void ObjectMoved(Rendering::SceneObject *object) = 0;
		virtual void ObjectAdded(Rendering::SceneObject *object) = 0;
		virtual void ObjectRemoved(Rendering::SceneObject *object) = 0;

		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo) { m_vao = vao; m_vbo = vbo; m_ibo = ibo; }
		void SetIndices(std::vector<GLuint> *indices) { m_indices = indices; }
		void SetVerts(std::vector<Rendering::VertexFormat> *verts) { m_verts = verts; }
		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }

		std::unordered_map<std::string , float> GetLastFrameCriteria() const { return m_lastFrameCriteria; }
		void SetLastFrameCriteria(std::unordered_map<std::string , float> val) { m_lastFrameCriteria = val; }

	protected:

		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() = 0;
		virtual void _Update() = 0;

		std::vector<Rendering::SceneObject *> *m_allObjects;
		bool m_showDebug;
		GLuint m_vao, m_vbo, m_ibo;
		std::vector<GLuint> *m_indices;
		std::vector<Rendering::VertexFormat> *m_verts;
		std::unordered_map<std::string , float> m_lastFrameCriteria;

		size_t m_lastFrameTests;
		size_t m_memoryUsed;

	private:

	};

	inline ICollisionMethod::~ICollisionMethod() {}


	inline std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> ICollisionMethod::TestCollision()
	{		
		auto start = std::chrono::high_resolution_clock::now();
		std::unordered_set<std::pair<SceneObject *, SceneObject *>> result = _TestCollision();
		auto end = std::chrono::high_resolution_clock::now();

		auto timeSpent = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

		m_lastFrameCriteria[Core::COLLISION_TIME] = ((float) timeSpent) / 1000000;
		m_lastFrameCriteria[Core::INTERSECTION_TESTS] = (float)m_lastFrameTests;
		m_lastFrameCriteria[Core::MEMORY] = ((float)m_memoryUsed) / 1024;

		return result;
	}

	inline void ICollisionMethod::Update()
	{
		auto start = std::chrono::high_resolution_clock::now();
		_Update();
		auto end = std::chrono::high_resolution_clock::now();

		auto timeSpent = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

		m_lastFrameCriteria[Core::STRUCTURE_TIME] = ((float) timeSpent) / 1000000;
	}

}
