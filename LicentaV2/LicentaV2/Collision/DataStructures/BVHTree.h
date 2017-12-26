#pragma once

#include "../../Rendering/SceneObject.h"
#include "../../Collision/DataStructures/BoundingBox.h"

namespace Collision
{
	namespace DataStructures
	{
		template <typename T>
		class BVHTree
		{
		public:
			enum NodeType {DEFAULT, NODE, LEAF};

			BVHTree();
			~BVHTree();
			bool IsLeaf();

			BVHTree<T> *m_left, *m_right;
			Collision::DataStructures::BoundingBox m_boundingBox;
			T *m_objects;
			size_t m_numObjects;
			NodeType m_type;
		private:
		};

		template <typename T>
		bool Collision::DataStructures::BVHTree<T>::IsLeaf()
		{
			return m_type == LEAF;
		}

		template <typename T>
		Collision::DataStructures::BVHTree<T>::~BVHTree()
		{
			delete m_left;
			delete m_right;
		}

		template <typename T>
		Collision::DataStructures::BVHTree<T>::BVHTree()
		{
			m_type = DEFAULT;
		}

	}
}