#include "SpatialGrid.h"
#include "../Rendering/VertexFormat.h"
#include "../Managers/ShaderManager.h"
#include "../Collision/DataStructures/BoundingBox.h"
#include "../Rendering/Models/Model.h"
#include "../Rendering/ShapeRenderer.h"

using namespace Rendering;

Collision::SpatialGrid::SpatialGrid(std::vector<Rendering::SceneObject *> *allObjects, int numberOfCells)
{
	m_allObjects = allObjects;
	m_numberOfCells = numberOfCells;

	MakeNewGrid();

	m_memoryUsed = sizeof(Collision::SpatialGrid) + sizeof(std::vector<Rendering::SceneObject *>) * m_numberOfCells * m_numberOfCells * m_numberOfCells;
}

void Collision::SpatialGrid::_Update()
{
	m_memoryUsed = sizeof(Collision::SpatialGrid);

	MakeNewGrid();

	m_memoryUsed += sizeof(std::vector<Rendering::SceneObject *>) * m_numberOfCells * m_numberOfCells * m_numberOfCells;

	for (auto currentObj : (*m_allObjects))
	{
		std::vector<glm::vec3> cellIndexes = FindCells(currentObj);
		for (int i = 0; i < cellIndexes.size(); ++i)
		{
			m_grid[static_cast<std::size_t>(cellIndexes[i].x)][static_cast<std::size_t>(cellIndexes[i].y)][static_cast<std::size_t>(cellIndexes[i].z)].push_back(currentObj);
		}
		m_memoryUsed += sizeof(Rendering::SceneObject*) * cellIndexes.size();
	}
}

void Collision::SpatialGrid::MakeNewGrid()
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
}

void Collision::SpatialGrid::DrawDebug(const glm::mat4& viewProjection)
{
	if (!GetShowDebug())
		return;

	for (int i = 0; i < m_numberOfCells; ++i)
	{
		for (int j = 0; j < m_numberOfCells; ++j)
		{
			for (int k = 0; k < m_numberOfCells; ++k)
			{
				glm::vec3 trans = m_worldMin + m_cellSize / 2.f + glm::vec3(i, j, k) * m_cellSize;
				glm::mat4 modelMatrix = glm::translate(glm::mat4(1), trans) * glm::scale(glm::mat4(1), m_cellSize);
				//glm::mat4 MVPMatrix = viewProjection * viewMatrix * modelMatrix;
				
				glLineWidth(static_cast<GLfloat>(m_grid[i][j][k].size() ? 3 : 1));
				Rendering::ShapeRenderer::DrawWithLines(viewProjection * modelMatrix, m_vao, *m_indices, m_grid[i][j][k].size() ? COLLISIONMETHOD : 0);
				glLineWidth(1);
			}
		}
	}
}

void Collision::SpatialGrid::SetParams(glm::vec3 worldMin, glm::vec3 worldMax, int numberOfCells)
{
	m_worldMin = worldMin;
	m_worldMax = worldMax;
	m_numberOfCells = numberOfCells;

	m_cellSize = (m_worldMax - m_worldMin) / ((float)m_numberOfCells);
}

std::vector<glm::vec3> Collision::SpatialGrid::FindCells(SceneObject *obj)
{
	Collision::DataStructures::BoundingBox objBB = ((Models::Model *)obj)->GetBoundingBox();

	std::vector<glm::vec3> result;

	glm::vec3 aux = -m_worldMin;
	int gridMinX = (int)glm::floor((objBB.m_minX + aux.x) / m_cellSize.x);
	int gridMinY = (int)glm::floor((objBB.m_minY + aux.y) / m_cellSize.y);
	int gridMinZ = (int)glm::floor((objBB.m_minZ + aux.z) / m_cellSize.z);
										 
	int gridMaxX = (int)glm::floor((objBB.m_maxX + aux.x) / m_cellSize.x);
	int gridMaxY = (int)glm::floor((objBB.m_maxY + aux.y) / m_cellSize.y);
	int gridMaxZ = (int)glm::floor((objBB.m_maxZ + aux.z) / m_cellSize.z);

	for (int i = gridMinX; i <= gridMaxX; ++i)
	{
		for (int j = gridMinY; j <= gridMaxY; ++j)
		{
			for (int k = gridMinZ; k <= gridMaxZ; ++k)
			{
				result.push_back(glm::vec3(i, j, k));
			}
		}
	}

	return result;
}

void Collision::SpatialGrid::ObjectMoved(Rendering::SceneObject *object)
{
}

void Collision::SpatialGrid::ObjectAdded(Rendering::SceneObject *object)
{
}

void Collision::SpatialGrid::ObjectRemoved(Rendering::SceneObject *object)
{
}

std::unordered_set<std::pair<SceneObject *, SceneObject *>> Collision::SpatialGrid::_TestCollision()
{
	std::unordered_set<std::pair<SceneObject *, SceneObject *>> result;
	m_lastFrameTests = 0;	

	for (auto obj : *(m_allObjects))
	{
		std::vector<glm::vec3> objCells = FindCells(obj);

		for (auto cellIndex : objCells)
		{
			for (auto secondObj : m_grid[(size_t)cellIndex.x][(size_t)cellIndex.y][(size_t)cellIndex.z])
			{
				if (obj == secondObj) continue;

				std::pair<SceneObject *, SceneObject *> firstPair(obj, secondObj);

				m_lastFrameTests++;
				if (((Models::Model *)obj)->GetBoundingBox().Collides(((Models::Model *)secondObj)->GetBoundingBox()))
				{
					result.insert(firstPair);
				}

			}
		}
	}

	return result;
}
