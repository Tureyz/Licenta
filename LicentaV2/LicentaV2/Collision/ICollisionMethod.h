#pragma once
#include <vector>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include "..\Core\Utils.hpp"
#include "..\Rendering\IPhysicsObject.h"
#include "..\Managers\ModelManager.h"



using namespace Rendering;


namespace std
{

	template <> struct hash<glm::vec3>
	{
		inline size_t operator()(const glm::vec3 &v) const {
			std::hash<size_t> hasher;
			return hasher((uint64_t)(v.x)) ^ hasher((uint64_t)(v.y)) ^ hasher((uint64_t)(v.z));
		}
	};
}

namespace Collision
{
	class ICollisionMethod
	{
	public:

		virtual ~ICollisionMethod() = 0;

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() final;
		virtual void Update() final;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectAdded(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) = 0;

		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo) { m_vao = vao; m_vbo = vbo; m_ibo = ibo; }
		void SetIndices(std::vector<GLuint> *indices) { m_indices = indices; }
		void SetVerts(std::vector<Rendering::VertexFormat> *verts) { m_verts = verts; }
		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }

		std::unordered_map<std::wstring , float> GetLastFrameCriteria() const { return m_lastFrameCriteria; }
		void SetLastFrameCriteria(std::unordered_map<std::wstring , float> val) { m_lastFrameCriteria = val; }

		Managers::ModelManager * GetModelManager() const { return m_modelManager; }
		void SetModelManager(Managers::ModelManager * val) { m_modelManager = val; }
	protected:

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() = 0;
		virtual void _Update() = 0;

		std::vector<Rendering::IPhysicsObject *> *m_allObjects;
		bool m_showDebug;
		GLuint m_vao, m_vbo, m_ibo;
		std::vector<GLuint> *m_indices;
		std::vector<Rendering::VertexFormat> *m_verts;
		std::unordered_map<std::wstring , float> m_lastFrameCriteria;

		size_t m_lastFrameTests;
		size_t m_memoryUsed;

		Managers::ModelManager *m_modelManager;
	private:

	};

	inline ICollisionMethod::~ICollisionMethod() {}


	inline std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> ICollisionMethod::TestCollision()
	{		
		auto start = std::chrono::high_resolution_clock::now();
		std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> result = _TestCollision();
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
