#pragma once
#include "Model.h"
namespace Rendering
{
	namespace Models
	{
		class BoundingBox;
		class Triangle : public Model
		{
		public:
			~Triangle();

			void Create();
			virtual void Update() override final;
			virtual void Draw() override final;
		private:
			Collision::DataStructures::BoundingBox *m_boundingBox;
		};
	}
}