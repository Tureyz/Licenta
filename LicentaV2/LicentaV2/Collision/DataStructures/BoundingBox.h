#pragma once
#include "../../Rendering/IPhysicsObject.h"

namespace Collision	
{
	namespace DataStructures {

		class BoundingBox
		{
		public:
			BoundingBox(Rendering::IPhysicsObject *parentObject);
			BoundingBox(Rendering::IPhysicsObject **parentObjects, size_t numObjects);
			BoundingBox();
			~BoundingBox();

			void Create();
			virtual void Update();
			virtual void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
			void Destroy();
			void UpdateValues();
			bool GetVisible() const { return m_isVisible; }
			void SetVisible(bool val) { m_isVisible = val; }
			void SetProgram(GLuint shaderName);
			bool Collides(const BoundingBox * other);
			GLfloat m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;

		private:
			bool m_isVisible;
			Rendering::IPhysicsObject * m_parentObject;
			Rendering::IPhysicsObject ** m_parentObjects;
			size_t m_numObjects;
			glm::vec4 m_color;
			GLuint m_vao;
			GLuint m_program;
			std::vector<GLuint> m_vbos;
		};
	}
}