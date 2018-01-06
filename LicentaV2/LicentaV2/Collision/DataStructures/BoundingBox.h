#pragma once
#include "../../Rendering/VisualBody.h"

namespace Collision	
{
	namespace DataStructures
	{

		class BoundingBox
		{
		public:
			BoundingBox();
			~BoundingBox();

			void CreateVisualBody(Rendering::VisualBody *visualBody);

			virtual void Update();

			void UpdateValues(std::vector<glm::vec3> coords);
			void UpdateValuesUnsorted(glm::vec3 p1, glm::vec3 p2);			
			void UpdateValuesUnsorted(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4);

			void UpdateValues(glm::vec3 minCoords, glm::vec3 maxCoords);
			void UpdateValues(std::vector<std::pair<glm::vec3, glm::vec3>> objectBounds);

			bool Collides(const BoundingBox &other) const;

			GLfloat m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
			float m_thickness;

			Rendering::VisualBody *m_visualBody;

			bool GetVisible() const { return m_isVisible; }
			void SetVisible(bool val) { m_isVisible = val; }
		private:
			bool m_isVisible;
			void Enlarge();
			void UpdateVisualVerts();
		};
	}
}