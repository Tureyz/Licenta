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
	m_test = new Collision::BVH(&m_physicsModelList);
// 	m_test->SetParams(glm::vec3(-5, -5, -5), glm::vec3(5, 5, 5), 5);
 	m_test->SetShowDebug(false);
	m_inited = false;
	m_objectIDCounter = 0;
	m_timeBase = m_time = 0;
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

	m_test->DrawDebug(projectionMatrix, viewMatrix);
}

void Managers::ModelManager::Update()
{
	Init();

	for (auto model : m_physicsModelList)
	{
		model->Update();
	}

	for (auto model : m_physicsModelListNDC)
	{
		model->Update();
	}

// 	if (!asdd) {
// 		TestCollision();
// 		asdd = true;
// 	}


	// Collision checks 30 times per second
	m_time = glutGet(GLUT_ELAPSED_TIME);
	if (m_time - m_timeBase > 1000.f / 30.f)
	{
		m_timeBase = m_time;
		//std::cout << "TICK\n";
		m_test->Update();
		TestCollision();
	}
}

void Managers::ModelManager::DeleteModel(unsigned long id)
{
	IPhysicsObject *model = NULL;
	for (std::vector<IPhysicsObject*>::iterator it = m_physicsModelList.begin(); it != m_physicsModelList.end(); ++it)
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
	IPhysicsObject *model = NULL;
	for (std::vector<IPhysicsObject*>::iterator it = m_physicsModelListNDC.begin(); it != m_physicsModelListNDC.end(); ++it)
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
	// TODO: insert return statement here
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

void Managers::ModelManager::SetModel(unsigned long id, IPhysicsObject *physicsObject)
{
	bool found = false;
	for (auto mod : m_physicsModelList)
	{
		if (mod->GetID() == id)
		{
			mod = physicsObject;
			found = true;
			break;
		}
	}

	if (!found) {
		physicsObject->SetID(id);
		m_physicsModelList.push_back(physicsObject);
	}
}

void Managers::ModelManager::SpawnObjectAt(const glm::vec3 &position, const physicsObjectType objectType, const glm::vec4 &color)
{
	

	if (objectType == OBJ_CUBE)
	{
		Rendering::Models::Cube *cube = new Rendering::Models::Cube(color, this);
		//cube->SetProgram(Managers::ShaderManager::GetShader("colorShader"));
		cube->SetID(m_objectIDCounter);
		cube->Create();
		glm::vec3 rot = glm::vec3((std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f);
		float angle = (float)(std::rand() % 360);
		//std::cout << "ROT: " << rot.x << " " << rot.y << " " << rot.z << " " << angle << std::endl;
		glm::vec3 scl = glm::vec3((std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f);
		cube->ScaleAbsolute(scl);
		cube->RotateAbsolute(rot, angle);
		cube->TranslateAbsolute(position);

		//std::cout << "scl: " << scl.x << " " << scl.y << " " << scl.z << std::endl;


		SetModel(m_objectIDCounter++, cube);
	}

	if (objectType == OBJ_TETRAHEDRON)
	{
		Rendering::Models::Tetrahedron *tetra = new Rendering::Models::Tetrahedron(color, this);
		//tetra->SetProgram(Managers::ShaderManager::GetShader("colorShader"));
		tetra->SetID(m_objectIDCounter);
		tetra->Create();
		glm::vec3 rot = glm::vec3((std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f);
		float angle = (float)(std::rand() % 360);
		//std::cout << "ROT: " << rot.x << " " << rot.y << " " << rot.z << " " << angle << std::endl;
		glm::vec3 scl = glm::vec3((std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f);
		tetra->ScaleAbsolute(scl);
		tetra->RotateAbsolute(rot, angle);
		tetra->TranslateAbsolute(position);

		//std::cout << "scl: " << scl.x << " " << scl.y << " " << scl.z << std::endl;


		SetModel(m_objectIDCounter++, tetra);
	}

	if (objectType == OBJ_SPHERE)
	{
		Rendering::Models::Sphere *sphere = new Rendering::Models::Sphere(color, this);
		//sphere->SetProgram(Managers::ShaderManager::GetShader("colorShader"));
		sphere->SetID(m_objectIDCounter);
		sphere->Create();
		sphere->TranslateAbsolute(position);
		glm::vec3 rot = glm::vec3((std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f, (std::rand() % 360) / 360.f);
		float angle = (float)(std::rand() % 360);
		//std::cout << "ROT: " << rot.x << " " << rot.y << " " << rot.z << " " << angle << std::endl;
		sphere->RotateAbsolute(rot, angle);
		glm::vec3 scl = glm::vec3((std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f, (std::rand() % 200) / 500.f);
		sphere->ScaleAbsolute(scl);

		//std::cout << "scl: " << scl.x << " " << scl.y << " " << scl.z << std::endl;


		SetModel(m_objectIDCounter++, sphere);
	}

}

void Managers::ModelManager::SpawnManyAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Managers::physicsObjectType typeOfObjects)
{
	float deviation = radius / 2;
	int diameter = (int)(radius * 2);


	if (typeOfObjects == OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{

			glm::vec3 pos = glm::vec3(position.x + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation,
				position.y + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation,
				position.z + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation);

			//std::cout << "X: " << pos.x << ", Y: " << pos.y << ", Z: " << pos.z << std::endl;
			SpawnObjectAt(pos, (physicsObjectType) (rand() % 3), glm::vec4(0.7, 0.7, 0.7, 1));
		}
	}
	else
	{

		for (int i = 0; i < numberOfObjects; ++i)
		{

			glm::vec3 pos = glm::vec3(position.x + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation,
				position.y + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation,
				position.z + ((std::rand() % (diameter * 1000)) / 1000.f) - deviation);

			//std::cout << "X: " << pos.x << ", Y: " << pos.y << ", Z: " << pos.z << std::endl;
			SpawnObjectAt(pos, typeOfObjects, glm::vec4(0.7, 0.7, 0.7, 1));
		}
	}

	
}

void Managers::ModelManager::TestCollision()
{

	// one to many

// 	auto asd = m_test->TestCollision(m_physicsModelList[0]);
// 	m_physicsModelList[0]->SetCollisionState(ACTIVE);
// 	for (auto obj : asd) {
// 		obj->SetCollisionState(COLLIDING);
// 	}

	//many to many

	std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> asd = m_test->TestCollision();
	for (int i = 0; i < asd.size(); ++i)
	{
		asd[i].first->SetCollisionState(COLLIDING);
		asd[i].second->SetCollisionState(COLLIDING);
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
	if (m_inited)
		return;

	CreateBufferObjects();

	SpawnManyAround(glm::vec3(0.f, 0.f, 0.f), 2.f, 200, Managers::physicsObjectType::OBJ_RANDOM);
	SetBoundingBoxesVisibile(true);

	m_inited = true;

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
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//right
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//back
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//left
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	//upper
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, 0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));


	//bottom
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, -0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_cubeVerts.push_back(VertexFormat(glm::vec3(-0.5, -0.5, 0.5), glm::vec4(0.7, 0.7, 0.7, 1)));

	glGenBuffers(1, &m_cubeVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_cubeVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(VertexFormat) * m_cubeVerts.size(), &m_cubeVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_cubeIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cubeIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_cubeIndices.size() * sizeof(unsigned int), &m_cubeIndices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)(offsetof(VertexFormat, VertexFormat::m_color)));
	glBindVertexArray(0);

	//////////////////////////////////////////////////////////

	// Tetrahedron

	glGenVertexArrays(1, &m_tetraVao);
	glBindVertexArray(m_tetraVao);
	m_tetraIndices = { 0, 1, 2, 3, 0, 1 };

	m_tetraVerts.push_back(VertexFormat(glm::vec3(1.0, 1.0, 1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(VertexFormat(glm::vec3(-1.0, -1.0, 1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(VertexFormat(glm::vec3(-1.0, 1.0, -1.0), glm::vec4(0.7, 0.7, 0.7, 1)));
	m_tetraVerts.push_back(VertexFormat(glm::vec3(1.0, -1.0, -1.0), glm::vec4(0.7, 0.7, 0.7, 1)));

	glGenBuffers(1, &m_tetraVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_tetraVbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(VertexFormat) * m_tetraVerts.size(), &m_tetraVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_tetraIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_tetraIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_tetraIndices.size() * sizeof(unsigned int), &m_tetraIndices[0], GL_STATIC_DRAW);


	// 	glGenBuffers(1, &tbo);
	// 	glBindBuffer(GL_ARRAY_BUFFER, tbo);
	// 	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * m_tetraVerts.size(), nullptr, GL_STATIC_READ);
	// 	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void *)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void *)(offsetof(VertexFormat, VertexFormat::m_color)));

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

			m_sphereVerts.push_back(VertexFormat(glm::vec3(x, y, z), glm::vec4(0.7, 0.7, 0.7, 1)));
		}
	}

	for (r = 0; r < longs - 1; r++)
	{
		for (s = 0; s < lats - 1; s++)
		{
			m_sphereIndices.push_back(r * lats + s);
			m_sphereIndices.push_back(r * lats + (s + 1));
			m_sphereIndices.push_back((r + 1) * lats + (s + 1));

			m_sphereIndices.push_back((r + 1) * lats + (s + 1));
			m_sphereIndices.push_back((r + 1) * lats + s);
			m_sphereIndices.push_back(r * lats + s);

		}
	}

	glGenVertexArrays(1, &m_sphereVao);
	glBindVertexArray(m_sphereVao);

	glGenBuffers(1, &m_sphereVbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_sphereVbo);
	glBufferData(GL_ARRAY_BUFFER, m_sphereVerts.size() * sizeof(VertexFormat), &m_sphereVerts[0], GL_STATIC_DRAW);

	glGenBuffers(1, &m_sphereIbo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphereIbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_sphereIndices.size() * sizeof(unsigned int), &m_sphereIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexFormat), (void*)(offsetof(VertexFormat, VertexFormat::m_color)));
	glBindVertexArray(0);


}
