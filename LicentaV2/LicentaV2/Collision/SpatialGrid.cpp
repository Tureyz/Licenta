#include "SpatialGrid.h"
#include "../Rendering/VertexFormat.h"
#include "../Managers/ShaderManager.h"
#include "../Collision/DataStructures/BoundingBox.h"
#include "../Rendering/Models/Model.h"

using namespace Rendering;

Collision::SpatialGrid::SpatialGrid(std::vector<Rendering::IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
}

std::vector<IPhysicsObject *> Collision::SpatialGrid::TestCollision(IPhysicsObject *queriedObject)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::SpatialGrid::Update()
{
	m_grid.clear();

	m_grid.resize(m_numberOfCells);
	for (int i = 0; i < m_numberOfCells; ++i)
	{
		m_grid[i].resize(m_numberOfCells);
		for (int j = 0; j < m_numberOfCells; ++j)
		{
			m_grid[i][j].resize(m_numberOfCells);
		}
	}

	for (auto currentObj : (*m_allObjects))
	{
		std::vector<glm::vec3> cellIndexes = FindCells(currentObj);
		for (int i = 0; i < cellIndexes.size(); ++i)
		{
			m_grid[cellIndexes[i].x][cellIndexes[i].y][cellIndexes[i].z].push_back(currentObj);
		}
	}
}

void Collision::SpatialGrid::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	
	if (!GetShowDebug())
		return;

	//For now, all objects use the same shaders
	//glUseProgram(Managers::ShaderManager::GetShader("colorShader"));
	

	for (int i = 0; i < m_numberOfCells; ++i)
	{
		for (int j = 0; j < m_numberOfCells; ++j)
		{
			for (int k = 0; k < m_numberOfCells; ++k)
			{
				glUniform1i(2, m_grid[i][j][k].size() ? BOUNDINGBOX : 0);
				glm::vec3 trans = m_worldMin + m_cellSize / 2.f + glm::vec3(i, j, k) * m_cellSize;
				glm::mat4 modelMatrix = glm::translate(glm::mat4(1), trans) * glm::scale(glm::mat4(1), m_cellSize);
				//std::cout << m_cellSize.x << " " << (m_worldMin + ((float)i) * m_cellSize).x << std::endl;
				glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * modelMatrix;
				glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);

				glBindVertexArray(m_vao);
				glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
			}
		}
	}

// 	glUniform1i(2, ACTIVE);
// 	glm::vec3 trans = m_worldMin + (m_worldMax - m_worldMin) / 2.f;
// 	glm::mat4 modelMatrix = glm::translate(glm::mat4(1), trans) * glm::scale(glm::mat4(1), (m_worldMax - m_worldMin));
// 	//std::cout << m_cellSize.x << " " << (m_worldMin + ((float)i) * m_cellSize).x << std::endl;
// 	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * modelMatrix;
// 	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);
// 
// 	glBindVertexArray(m_vao);
// 	//glDrawArrays(GL_TRIANGLES, 0, 36);
// 	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

void Collision::SpatialGrid::SetParams(glm::vec3 worldMin, glm::vec3 worldMax, int numberOfCells)
{
	m_worldMin = worldMin;
	m_worldMax = worldMax;
	m_numberOfCells = numberOfCells;

	m_cellSize = (m_worldMax - m_worldMin) / ((float)m_numberOfCells);

}

std::vector<glm::vec3> Collision::SpatialGrid::FindCells(IPhysicsObject *obj)
{
	Collision::DataStructures::BoundingBox *objBB = ((Models::Model *)obj)->GetBoundingBox();

	//glm::vec3 gridMin = glm::floor((glm::vec3(objBB->m_minX, objBB->m_minY, objBB->m_minZ) - m_worldMin + m_cellSize / 2.f) / m_cellSize);
	//glm::vec3 gridMax = glm::floor((glm::vec3(objBB->m_maxX, objBB->m_maxY, objBB->m_maxZ) - m_worldMin + m_cellSize / 2.f) / m_cellSize);

	std::vector<glm::vec3> result;
	 
	glm::vec3 aux = -m_worldMin;
	int gridMinX = (int) glm::floor((objBB->m_minX + aux.x) / m_cellSize.x);
	int gridMinY = (int) glm::floor((objBB->m_minY + aux.y) / m_cellSize.y);
	int gridMinZ = (int) glm::floor((objBB->m_minZ + aux.z) / m_cellSize.z);
	 
	int gridMaxX = (int) glm::floor((objBB->m_maxX + aux.x) / m_cellSize.x);
	int gridMaxY = (int) glm::floor((objBB->m_maxY + aux.y) / m_cellSize.y);
	int gridMaxZ = (int) glm::floor((objBB->m_maxZ + aux.z) / m_cellSize.z);

// 	for (int i = gridMin.x; i <= gridMax.x; ++i)
// 	{
// 		for (int j = gridMin.y; j <= gridMax.y; ++j)
// 		{
// 			for (int k = gridMin.z; k <= gridMax.z; ++k)
// 			{
// 				//m_grid[i][j][k].push_back(obj);
// 				result.push_back(glm::vec3(i, j, k));
// 			}
// 		}
// 	}

	for (int i = gridMinX; i <= gridMaxX; ++i)
	{
		for (int j = gridMinY; j <= gridMaxY; ++j)
		{
			for (int k = gridMinZ; k <= gridMaxZ; ++k)
			{
				//m_grid[i][j][k].push_back(obj);
				result.push_back(glm::vec3(i, j, k));
			}
		}
	}

	return result;
}

std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> Collision::SpatialGrid::TestCollision()
{
	std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> result;

	//m_checkedPairs.clear();

	size_t breakCount = m_allObjects->size() * m_allObjects->size();
	for (auto obj : *(m_allObjects))
	{
		std::vector<glm::vec3> objCells = FindCells(obj);

 		for (auto cellIndex : objCells)
 		{
 			for (auto secondObj : m_grid[(size_t)cellIndex.x][(size_t)cellIndex.y][(size_t)cellIndex.z])
 			{
 				if (obj == secondObj)
 					continue;

				std::pair<IPhysicsObject *, IPhysicsObject *> firstPair = std::make_pair(obj, secondObj);
//				std::pair<IPhysicsObject *, IPhysicsObject *> secondPair = std::make_pair(secondObj, obj);

// 				if (!m_checkedPairs[firstPair] && !m_checkedPairs[secondPair])
// 				{
					if (((Models::Model *)obj)->GetBoundingBox()->Collides(((Models::Model *)secondObj)->GetBoundingBox()))
					{
						result.push_back(firstPair);
					}
// 				}
//  				if (m_checkedPairs.size() == breakCount)
//  				{
//  					return result;
//  				}
 			}
 		}
	}

	return result;
}
