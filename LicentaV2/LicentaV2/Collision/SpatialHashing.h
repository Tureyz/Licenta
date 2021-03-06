#pragma once
#include "ICollisionMethod.h"
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <bitset>

namespace Collision
{
	class SpatialHashing : public ICollisionMethod
	{
	public:

		SpatialHashing(std::vector<Rendering::SceneObject *> *allObjects);

		~SpatialHashing();

		virtual void DrawDebug(const glm::mat4& viewProjection) override;
		
		float GetCellSize() const { return m_cellSize; }
		void SetCellSize(float val) { m_cellSize = val; }
		static std::wstring  BinaryToString(short int x);
		static std::wstring  BinaryToString(size_t x);

	protected:

		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		void InsertPoint(glm::vec3 point, Rendering::SceneObject *obj);
		void InsertObject(Rendering::SceneObject *obj);

		void RemoveObject(Rendering::SceneObject *obj);

		void MoveObject(Rendering::SceneObject *obj);

		size_t ObjectHash(glm::vec3 el);

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

		float m_cellSize;
		std::vector<std::unordered_set<SceneObject*>> m_hashTable;
		std::unordered_map<size_t, size_t> m_bucketIndices;
	};
}