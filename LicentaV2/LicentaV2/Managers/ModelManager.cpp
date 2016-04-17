#include "ModelManager.h"
#include "../Rendering/Models/Triangle.h"
#include "../Rendering/Models/Quad.h"
#include "../Rendering/Models/Cube.h"
#include "../Rendering/Models/Tetrahedron.h"
#include "../Rendering/Models/Sphere.h"

#include <utility>
#include <string>
#include <cstdlib>

Managers::ModelManager::ModelManager()
{
}

Managers::ModelManager::~ModelManager()
{
	for (auto model : m_physicsModelList)
	{
		delete model;
	}
	m_physicsModelList.clear();

	for (auto model : m_physicsModelListNDC)
	{
		delete model;
	}
	m_physicsModelListNDC.clear();
}

void Managers::ModelManager::Draw()
{
	for (auto model : m_physicsModelListNDC)
	{
		model->Draw();
	}
}

void Managers::ModelManager::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	for (auto model : m_physicsModelList)
	{
		model->Draw(projectionMatrix, viewMatrix);
	}
}

void Managers::ModelManager::Update()
{
	for (auto model : m_physicsModelList)
	{
		model->Update();
	}

	for (auto model : m_physicsModelListNDC)
	{
		model->Update();
	}
}

void Managers::ModelManager::DeleteModel(unsigned long id)
{
	Rendering::IPhysicsObject *model = NULL;
	for (std::vector<Rendering::IPhysicsObject*>::iterator it = m_physicsModelList.begin(); it != m_physicsModelList.end(); ++it)
	{
		if ((*it)->GetID() == id)
		{
			(*it)->Destroy();
			m_physicsModelList.erase(it);
			break;
		}
	}
}

void Managers::ModelManager::DeleteModelNDC(unsigned long id)
{
	Rendering::IPhysicsObject *model = NULL;
	for (std::vector<Rendering::IPhysicsObject*>::iterator it = m_physicsModelListNDC.begin(); it != m_physicsModelListNDC.end(); ++it)
	{
		if ((*it)->GetID() == id)
		{
			(*it)->Destroy();
			m_physicsModelListNDC.erase(it);
			break;
		}
	}
}

const Rendering::IPhysicsObject* Managers::ModelManager::GetModel(unsigned long id) const
{
	for (auto mod : m_physicsModelList)
	{
		if (mod->GetID() == id)
		{
			return mod;
		}
	}

	return NULL;
}

const Rendering::IPhysicsObject* Managers::ModelManager::GetModelNDC(unsigned long id) const
{
	for (auto mod : m_physicsModelListNDC)
	{
		if (mod->GetID() == id)
		{
			return mod;
		}
	}

	return NULL;
}

void Managers::ModelManager::SetModel(size_t id, Rendering::IPhysicsObject *gameObject)
{
	bool found = false;
	for (auto mod : m_physicsModelList)
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
		m_physicsModelList.push_back(gameObject);
	}
}

void Managers::ModelManager::SetBoundingBoxesVisibile(bool value)
{
	for (auto obj : m_physicsModelList)
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

	// CUBE

	glGenVertexArrays(1, &m_cubeVao);
	glBindVertexArray(m_cubeVao);

	m_cubeIndices = { 0, 1, 2, 0, 2, 3, //front
		4, 5, 6, 4, 6, 7, //right
		8, 9, 10, 8, 10, 11, //back
		12, 13, 14, 12, 14, 15, //left
		16, 17, 18, 16, 18, 19, //upper
		20, 21, 22, 20, 22, 23 }; //bottom

								  //front
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//right
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//back
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//left
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//upper
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));


	//bottom
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(Rendering::VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	glGenBuffers(1, &m_cubeVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * m_cubeVerts.size(), &m_cubeVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_cubeIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cubeIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_cubeIndices.size() * sizeof(unsigned int), &m_cubeIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void*)(offsetof(Rendering::VertexFormat, Rendering::VertexFormat::m_color)));
	glBindVertexArray(0);

	//////////////////////////////////////////////////////////

	// Tetrahedron

	glGenVertexArrays(1, &m_tetraVao);
	glBindVertexArray(m_tetraVao);
	m_tetraIndices = { 0, 1, 2, 3, 0, 1 };

	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, 1.0, 1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, -1.0, 1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, 1.0, -1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, -1.0, -1.0), glm::vec4(0.7, 0.7, 0.7, 1)));

	glGenBuffers(1, &m_tetraVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_tetraVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Rendering::VertexFormat) * m_tetraVerts.size(), &m_tetraVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_tetraIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetraIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tetraIndices.size() * sizeof(unsigned int), &m_tetraIndices[0], GL_STATIC_DRAW);


	// 	glGenBuffers(1, &tbo);
	// 	glBindBuffer(GL_ARRAY_BUFFER, tbo);
	// 	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * m_tetraVerts.size(), nullptr, GL_STATIC_READ);
	// 	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void *)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void *)(offsetof(Rendering::VertexFormat, Rendering::VertexFormat::m_color)));

	glBindVertexArray(0);

	//////////////////////////////////////////////////////////

	//Sphere

	float longs = 15, lats = 15;
	float const R = 1.f / (float)(longs - 1);
	float const S = 1.f / (float)(lats - 1);
	int r, s;

	//vertices.resize(m_longs * m_lats * 3);

	for (r = 0; r < longs; r++)
	{
		for (s = 0; s < lats; s++)
		{
			float y = sin(-glm::half_pi<float>() + glm::pi<float>()* r * R);
			float x = cos(2 * glm::pi<float>() * s * S) * sin(glm::pi<float>() * r * R);
			float z = sin(2 * glm::pi<float>() * s * S) * sin(glm::pi<float>() * r * R);

			m_sphereVerts.push_back(Rendering::VertexFormat(glm::vec3(x, y, z), glm::vec4(0.7, 0.7, 0.7, 1)));
		}
	}

	for (r = 0; r < longs - 1; r++)
	{
		for (s = 0; s < lats - 1; s++)
		{
			m_sphereIndices.push_back((GLuint)(r * lats + s));
			m_sphereIndices.push_back((GLuint)(r * lats + (s + 1)));
			m_sphereIndices.push_back((GLuint)((r + 1) * lats + (s + 1)));

			m_sphereIndices.push_back((GLuint)((r + 1) * lats + (s + 1)));
			m_sphereIndices.push_back((GLuint)((r + 1) * lats + s));
			m_sphereIndices.push_back((GLuint)(r * lats + s));

		}
	}

	glGenVertexArrays(1, &m_sphereVao);
	glBindVertexArray(m_sphereVao);

	glGenBuffers(1, &m_sphereVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_sphereVbo);
	glBufferData(GL_ARRAY_BUFFER, m_sphereVerts.size() * sizeof(Rendering::VertexFormat), &m_sphereVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_sphereIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphereIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_sphereIndices.size() * sizeof(unsigned int), &m_sphereIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Rendering::VertexFormat), (void*)(offsetof(Rendering::VertexFormat, Rendering::VertexFormat::m_color)));
	glBindVertexArray(0);


}
