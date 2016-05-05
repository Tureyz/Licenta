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

		SpatialHashing(std::vector<Rendering::IPhysicsObject *> *allObjects);
		~SpatialHashing();

		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		
		float GetCellSize() const { return m_cellSize; }
		void SetCellSize(float val) { m_cellSize = val; }
		static std::string BinaryToString(short int x);
		static std::string BinaryToString(size_t x);
	private:

		void InsertPoint(glm::vec3 point, Rendering::IPhysicsObject *obj);
		void InsertObject(Rendering::IPhysicsObject *obj);

		void RemoveObject(Rendering::IPhysicsObject *obj);

		void MoveObject(Rendering::IPhysicsObject *obj);

		size_t ObjectHash(glm::vec3 el);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		float m_cellSize;
		std::vector<std::unordered_set<IPhysicsObject*>> m_hashTable;
		std::unordered_map<size_t, size_t> m_bucketIndices;
	};
}