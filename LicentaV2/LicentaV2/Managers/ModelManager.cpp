#include "ModelManager.h"
#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Tetrahedron.h"
#include "../Rendering/Models/Sphere.h"

#include <utility>
#include <string>
#include <cstdlib>
#include "../Core/Utils.hpp"

#include "../Rendering/ShapeRenderer.h"

Managers::ModelManager::ModelManager()
{
}

Managers::ModelManager::~ModelManager()
{
	for (auto model : m_objectList)
	{
		delete model;
	}
	m_objectList.clear();
}

void Managers::ModelManager::FixedUpdate()
{
	for (auto model : m_objectList)
	{
		model->FixedUpdate();
	}
}

void Managers::ModelManager::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	for (auto model : m_objectList)
	{
		model->Draw(projectionMatrix, viewMatrix);
	}
}

void Managers::ModelManager::Update()
{
	for (auto model : m_objectList)
	{
		model->Update();
	}
}

void Managers::ModelManager::DeleteAllModels()
{
	if (!m_objectList.empty())
	{
		for (auto model : m_objectList)
		{
			delete model;
		}

		m_objectList.clear();
	}
}

void Managers::ModelManager::DeleteModel(unsigned long id)
{
	Rendering::IPhysicsObject *model = NULL;
	for (auto it = m_objectList.begin(); it != m_objectList.end(); ++it)
	{
		if ((*it)->GetID() == id)
		{
			(*it)->Destroy();
			m_objectList.erase(it);
			break;
		}
	}
}

const Rendering::IPhysicsObject* Managers::ModelManager::GetModel(unsigned long id) const
{
	for (auto mod : m_objectList)
	{
		if (mod->GetID() == id)
		{
			return mod;
		}
	}

	return NULL;
}

void Managers::ModelManager::RegisterObject(size_t id, Rendering::IPhysicsObject *gameObject)
{
	bool found = false;
	for (auto mod : m_objectList)
	{
		if (mod->GetID() == id)
		{
			mod = gameObject;
			found = true;
			break;
		}
	}

	if (!found) {
		gameObject->SetID(id);
		m_objectList.push_back(gameObject);
	}
}

void Managers::ModelManager::SetBoundingBoxesVisibile(bool value)
{
	for (auto obj : m_objectList)
	{
		((Rendering::Models::Model *) obj)->SetBoundingBoxVisible(value);
	}
}

void Managers::ModelManager::Init()
{
	CreateBufferObjects();
}

void Managers::ModelManager::CreateBufferObjects()
{
	CreateCubeProps();
	Rendering::ShapeRenderer::CreateBufferObjects(m_cubeVao, m_cubeVbo, m_cubeIbo, m_cubeVerts, m_cubeIndices);

	CreateTetrahedronProps();
	Rendering::ShapeRenderer::CreateBufferObjects(m_tetraVao, m_tetraVbo, m_tetraIbo, m_tetraVerts, m_tetraIndices);

	CreateSphereProps();
	Rendering::ShapeRenderer::CreateBufferObjects(m_sphereVao, m_sphereVbo, m_sphereIbo, m_sphereVerts, m_sphereIndices);

	CreateCylinderProps();
	Rendering::ShapeRenderer::CreateBufferObjects(m_cylinderVao, m_cylinderVbo, m_cylinderIbo, m_cylinderVerts, m_cylinderIndices);

	CreateConeProps();
	Rendering::ShapeRenderer::CreateBufferObjects(m_coneVao, m_coneVbo, m_coneIbo, m_coneVerts, m_coneIndices);
}

void Managers::ModelManager::CreateCubeProps()
{
	m_cubeIndices = { 0, 1, 2, 0, 2, 3, //front
		4, 5, 6, 4, 6, 7, //right
		8, 9, 10, 8, 10, 11, //back
		12, 13, 14, 12, 14, 15, //left
		16, 17, 18, 16, 18, 19, //upper
		20, 21, 22, 20, 22, 23 }; //bottom	

	m_lineCubeIndices = { 0, 1, 2, 3, 0, 8, 6, 1, 2, 5, 6, 8, 11, 3, 11, 5, 6, 1, 0};

	//front
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));

	//right
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));

	//back
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));

	//left
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));

	//upper
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));

	//bottom
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), Core::DEFAULT_OBJECT_COLOR));
}

void Managers::ModelManager::CreateTetrahedronProps()
{
	m_tetraIndices = { 0, 1, 2,
		0, 1, 3,
		0, 2, 3 };

	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, 1.0, 1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, -1.0, 1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, 1.0, -1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, -1.0, -1.0), Core::DEFAULT_OBJECT_COLOR));
}

void Managers::ModelManager::CreateConeProps()
{
	float lats = 10, longs = 3;

	float th = 0.f;
	const float angleStep = glm::two_pi<float>() / lats;
	const float zStep = 1 / longs;
	const float rStep = 1 / longs;

	float r0 = 1.f, r1 = r0 - rStep;
	float z0 = 0.f, z1 = zStep;

	unsigned int crtIdx = 0;

	for (int i = 0; i < longs - 1; ++i)
	{
		for (int j = 0; j < lats; ++j)
		{
			th = j * angleStep;

			m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r0, glm::sin(th) * r0, z0), Core::DEFAULT_OBJECT_COLOR));
			m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r1, glm::sin(th) * r1, z1), Core::DEFAULT_OBJECT_COLOR));

			th = (j + 1) * angleStep;

			m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r0, glm::sin(th) * r0, z0), Core::DEFAULT_OBJECT_COLOR));
			m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r1, glm::sin(th) * r1, z1), Core::DEFAULT_OBJECT_COLOR));

			m_coneIndices.push_back(crtIdx);
			m_coneIndices.push_back(crtIdx + 2);
			m_coneIndices.push_back(crtIdx + 3);

			m_coneIndices.push_back(crtIdx);
			m_coneIndices.push_back(crtIdx + 3);
			m_coneIndices.push_back(crtIdx + 1);

			crtIdx += 4;
		}

		z0 = z1;
		z1 += zStep;
		r0 = r1;
		r1 -= rStep;
	}

	for (int i = 0; i < lats; ++i)
	{
		th = i * angleStep;

		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r0, glm::sin(th) * r0, z0), Core::DEFAULT_OBJECT_COLOR));
		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th) * r0, glm::sin(th) * r0, z0), Core::DEFAULT_OBJECT_COLOR));
		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(0, 0, 1), Core::DEFAULT_OBJECT_COLOR));

		m_coneIndices.push_back(crtIdx);
		m_coneIndices.push_back(crtIdx + 1);
		m_coneIndices.push_back(crtIdx + 2);

		crtIdx += 3;
	}

	for (int j = lats; j >= 0; j--)
	{
		th = j * angleStep;
		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th), glm::sin(th), 0), Core::DEFAULT_OBJECT_COLOR));

		th = (j - 1) * angleStep;
		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(glm::cos(th), glm::sin(th), 0), Core::DEFAULT_OBJECT_COLOR));

		m_coneVerts.push_back(Rendering::VertexFormat(glm::vec3(0, 0, 0), Core::DEFAULT_OBJECT_COLOR));

		m_coneIndices.push_back(crtIdx);
		m_coneIndices.push_back(crtIdx + 1);
		m_coneIndices.push_back(crtIdx + 2);

		crtIdx += 3;
	}
}

void Managers::ModelManager::CreateCylinderProps()
{
	float lats = 11, longs = 5;

	float heightStep = 1.0f / longs;
	float height = -heightStep;
	float deg = 0;

	m_cylinderVerts.resize(lats * longs);

	for (int i = 0; i < longs; ++i)
	{
		height += heightStep;

		for (int j = 0; j < lats; ++j)
		{
			m_cylinderVerts[i * lats + j] = Rendering::VertexFormat(glm::vec3(glm::cos(glm::radians(deg)), height, glm::sin(glm::radians(deg))), Core::DEFAULT_OBJECT_COLOR);
			deg += 360.f / (lats - 1);
		}
		deg = 0;
	}

	m_cylinderIndices.resize(6 * (longs - 1) * lats);
	int off = 0;

	for (int i = 0; i < longs - 1; ++i)
	{
		for (int j = 0; j < lats; ++j)
		{
			m_cylinderIndices[off] = i * lats + j;
			m_cylinderIndices[off + 1] = (i + 1) * lats + j;
			m_cylinderIndices[off + 2] = i * lats + j + 1;

			m_cylinderIndices[off + 3] = i * lats + j + 1;
			m_cylinderIndices[off + 4] = (i + 1) * lats + j;
			m_cylinderIndices[off + 5] = (i + 1) * lats + j + 1;
			off += 6;
		}
	}

	m_cylinderIndices.pop_back();
}

void Managers::ModelManager::CreateSphereProps()
{
	float longs = 15, lats = 15;
	float const R = 1.f / (float)(longs - 1);
	float const S = 1.f / (float)(lats - 1);
	int r, s;

	for (r = 0; r < longs; r++)
	{
		for (s = 0; s < lats; s++)
		{
			float y = sin(-glm::half_pi<float>() + glm::pi<float>()* r * R);
			float x = cos(2 * glm::pi<float>() * s * S) * sin(glm::pi<float>() * r * R);
			float z = sin(2 * glm::pi<float>() * s * S) * sin(glm::pi<float>() * r * R);

			m_sphereVerts.push_back(Rendering::VertexFormat(glm::vec3(x, y, z), Core::DEFAULT_OBJECT_COLOR));
		}
	}

	for (r = 0; r < longs - 1; r++)
	{
		for (s = 0; s < lats - 1; s++)
		{
			m_sphereIndices.push_back((unsigned int)(r * lats + s));
			m_sphereIndices.push_back((unsigned int)(r * lats + (s + 1)));
			m_sphereIndices.push_back((unsigned int)((r + 1) * lats + (s + 1)));

			m_sphereIndices.push_back((unsigned int)((r + 1) * lats + (s + 1)));
			m_sphereIndices.push_back((unsigned int)((r + 1) * lats + s));
			m_sphereIndices.push_back((unsigned int)(r * lats + s));
		}
	}
}
