#include <iostream>

#include "VisualBodyFactory.h"
#include "ShapeRenderer.h"
#include "../Core/Utils.hpp"

Rendering::VisualBodyFactory &Rendering::VisualBodyFactory::GetInstance()
{
	static VisualBodyFactory theInstance;
	return theInstance;
}

Rendering::VisualBody *Rendering::VisualBodyFactory::CreateBasicVisualBody(enum Rendering::VisualBodyType type)
{
	GLuint vao, vbo, ibo;

	switch (type)
	{
	case Rendering::VisualBodyType::OBJ_CUBE:
		CreateCubeBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_cubeVerts, m_cubeIndices, vao, vbo, ibo);
	case Rendering::VisualBodyType::OBJ_SPHERE:
		CreateSphereBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_sphereVerts, m_sphereIndices, vao, vbo, ibo);
	case Rendering::VisualBodyType::OBJ_TETRAHEDRON:
		CreateTetrahedronBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_tetraVerts, m_tetraIndices, vao, vbo, ibo);
	case Rendering::VisualBodyType::OBJ_CYLINDER:
		CreateCylinderBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_cylinderVerts, m_cylinderIndices, vao, vbo, ibo);
	case Rendering::VisualBodyType::OBJ_CONE:
		CreateConeBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_coneVerts, m_coneIndices, vao, vbo, ibo);
	case Rendering::VisualBodyType::OBJ_LINE_CUBE:
		CreateCubeBufferObjects(vao, vbo, ibo);
		return new Rendering::VisualBody(m_cubeVerts, m_lineCubeIndices, vao, vbo, ibo);
	default:
		std::cout << "Ai grijeeeee" << std::endl;
		return new Rendering::VisualBody();
	}
}

Rendering::VisualBody *Rendering::VisualBodyFactory::CreateMeshVisualBody(const int rows, const int cols)
{
	GLuint vao, vbo, ibo;
	auto props = CreateMeshProps(rows, cols);


	CreateMeshBufferObjects(rows, cols, vao, vbo, ibo, props.first, props.second);

	Rendering::VisualBody *result = new Rendering::VisualBody(vao, vbo, ibo);
	//	result.m_initialVerts = props.first;
	result->m_verts = props.first;
	result->m_indices = props.second;

	return result;
}

Rendering::VisualBodyFactory::VisualBodyFactory()
{
	CreateProps();
}

void Rendering::VisualBodyFactory::CreateProps()
{
	CreateSphereProps();
	CreateCubeProps();
	CreateTetrahedronProps();
	CreateCylinderProps();
	CreateConeProps();
}

void Rendering::VisualBodyFactory::CreateCubeProps()
{
	m_cubeIndices = { 0, 1, 2, 0, 2, 3, //front
		4, 5, 6, 4, 6, 7, //right
		8, 9, 10, 8, 10, 11, //back
		12, 13, 14, 12, 14, 15, //left
		16, 17, 18, 16, 18, 19, //upper
		20, 21, 22, 20, 22, 23 }; //bottom	

	m_lineCubeIndices = { 0, 1, 2, 3, 0, 8, 6, 1, 2, 5, 6, 8, 11, 3, 11, 5, 6, 1, 0 };

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

void Rendering::VisualBodyFactory::CreateTetrahedronProps()
{
	m_tetraIndices = { 0, 1, 2,
		0, 1, 3,
		0, 2, 3 };

	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, 1.0, 1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, -1.0, 1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(-1.0, 1.0, -1.0), Core::DEFAULT_OBJECT_COLOR));
	m_tetraVerts.push_back(Rendering::VertexFormat(glm::vec3(1.0, -1.0, -1.0), Core::DEFAULT_OBJECT_COLOR));
}

void Rendering::VisualBodyFactory::CreateConeProps()
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

	for (int j = (int)lats; j >= 0; j--)
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

void Rendering::VisualBodyFactory::CreateCylinderProps()
{
	float lats = 11, longs = 5;

	float heightStep = 1.0f / longs;
	float height = -heightStep;
	float deg = 0;

	m_cylinderVerts.resize((size_t)(lats * longs));

	for (int i = 0; i < longs; ++i)
	{
		height += heightStep;

		for (int j = 0; j < lats; ++j)
		{
			m_cylinderVerts[i * static_cast<uint64_t>(lats) + j] = Rendering::VertexFormat(glm::vec3(glm::cos(glm::radians(deg)), height, glm::sin(glm::radians(deg))), Core::DEFAULT_OBJECT_COLOR);
			deg += 360.f / (lats - 1);
		}
		deg = 0;
	}

	m_cylinderIndices.resize(6 * (static_cast<uint64_t>(longs) - 1) * static_cast<uint64_t>(lats));
	int off = 0;

	for (int i = 0; i < longs - 1; ++i)
	{
		for (int j = 0; j < lats; ++j)
		{
			m_cylinderIndices[off] = (GLuint)(i * lats + j);
			m_cylinderIndices[off + 1] = (GLuint)((i + 1) * lats + j);
			m_cylinderIndices[off + 2] = (GLuint)(i * lats + j + 1);

			m_cylinderIndices[off + 3] = (GLuint)(i * lats + j + 1);
			m_cylinderIndices[off + 4] = (GLuint)((i + 1) * lats + j);
			m_cylinderIndices[off + 5] = (GLuint)((i + 1) * lats + j + 1);
			off += 6;
		}
	}

	m_cylinderIndices.pop_back();
}

void Rendering::VisualBodyFactory::CreateSphereProps()
{
	const float X = .525731112119133606f;
	const float Z = .850650808352039932f;
	const int divisionDepth = 3;

	std::vector<glm::vec3> icoVerts = {
		glm::vec3(-X, 0, Z), glm::vec3(X, 0, Z), glm::vec3(-X, 0, -Z), glm::vec3(X, 0, -Z),
		glm::vec3(0, Z, X), glm::vec3(0, Z, -X), glm::vec3(0, -Z, X), glm::vec3(0, -Z, -X),
		glm::vec3(Z, X, 0), glm::vec3(-Z, X, 0), glm::vec3(Z, -X, 0), glm::vec3(-Z, -X, 0)
	};

	std::vector<unsigned int> icoIndices = {
		0, 4, 1, 0, 9, 4, 9, 5, 4, 4, 5, 8, 4, 8, 1,
		8, 10, 1, 8, 3, 10, 5, 3, 8, 5, 2, 3, 2, 7, 3,
		7, 10, 3, 7, 6, 10, 7, 11, 6, 11, 0, 6, 0, 1, 6,
		6, 1, 10, 9, 0, 11, 9, 11 ,2, 9, 2, 5, 7, 2, 11
	};

	for (int i = 0; i < divisionDepth; ++i)
	{
		std::vector<glm::vec3> newVerts;
		std::vector<unsigned int> newIndices;
		for (int j = 0; j < icoIndices.size(); j += 3)
		{
			SubdivideTriangle(icoVerts[icoIndices[j]], icoVerts[icoIndices[j + 1]], icoVerts[icoIndices[j + 2]], newVerts, newIndices);
		}

		icoVerts = newVerts;
		icoIndices = newIndices;
	}

	for (auto vert : icoVerts)
	{
		m_sphereVerts.push_back(Rendering::VertexFormat(vert, Core::DEFAULT_OBJECT_COLOR));
	}

	m_sphereIndices = icoIndices;
}

void Rendering::VisualBodyFactory::SubdivideTriangle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, std::vector<glm::vec3> &verts, std::vector<unsigned int> &indices)
{
	glm::vec3 v12 = glm::normalize(v1 + v2);
	glm::vec3 v23 = glm::normalize(v2 + v3);
	glm::vec3 v31 = glm::normalize(v3 + v1);

	verts.push_back(v1);
	unsigned int i1 = (unsigned int) verts.size() - 1;
	verts.push_back(v2);
	unsigned int i2 = (unsigned int) verts.size() - 1;
	verts.push_back(v3);
	unsigned int i3 = (unsigned int) verts.size() - 1;

	verts.push_back(v12);
	unsigned int i12 = (unsigned int) verts.size() - 1;
	verts.push_back(v23);
	unsigned int i23 = (unsigned int) verts.size() - 1;
	verts.push_back(v31);
	unsigned int i31 = (unsigned int) verts.size() - 1;

	indices.push_back(i1);
	indices.push_back(i12);
	indices.push_back(i31);

	indices.push_back(i2);
	indices.push_back(i23);
	indices.push_back(i12);

	indices.push_back(i3);
	indices.push_back(i31);
	indices.push_back(i23);

	indices.push_back(i12);
	indices.push_back(i23);
	indices.push_back(i31);
}

std::pair<std::vector<Rendering::VertexFormat>, std::vector<GLuint>> Rendering::VisualBodyFactory::CreateMeshProps(int rows, int cols)
{
	std::pair<std::vector<Rendering::VertexFormat>, std::vector<GLuint>> result;

	std::vector<Rendering::VertexFormat> verts;
	std::vector<GLuint> indices;

	float wStep = 2.0f / cols;
	float hStep = 2.0f / rows;

	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			verts.push_back(Rendering::VertexFormat(glm::vec3(hStep * i, wStep * j, 0), Core::DEFAULT_OBJECT_COLOR));
		}
	}

	for (int i = 0; i < rows - 1; ++i)
	{
		for (int j = 0; j < cols - 1; ++j)
		{
			indices.push_back(rows * i + j);
			indices.push_back(rows * (i + 1) + j + 1);
			indices.push_back(rows * i + j + 1);

			indices.push_back(rows * i + j);
			indices.push_back(rows * (i + 1) + j);
			indices.push_back(rows * (i + 1) + j + 1);
		}
	}

	return std::make_pair(verts, indices);
}

void Rendering::VisualBodyFactory::CreateMeshBufferObjects(const int rows, const int cols, GLuint &vao, GLuint &vbo, GLuint &ibo, std::vector<Rendering::VertexFormat> &verts, std::vector<GLuint> &indices)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, verts, indices);
}

void Rendering::VisualBodyFactory::CreateCubeBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, m_cubeVerts, m_cubeIndices);
}

void Rendering::VisualBodyFactory::CreateTetrahedronBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, m_tetraVerts, m_tetraIndices);
}

void Rendering::VisualBodyFactory::CreateConeBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, m_coneVerts, m_coneIndices);
}

void Rendering::VisualBodyFactory::CreateCylinderBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, m_cylinderVerts, m_cylinderIndices);
}

void Rendering::VisualBodyFactory::CreateSphereBufferObjects(GLuint &vao, GLuint &vbo, GLuint &ibo)
{
	Rendering::ShapeRenderer::CreateBufferObjects(vao, vbo, ibo, m_sphereVerts, m_sphereIndices);
}
