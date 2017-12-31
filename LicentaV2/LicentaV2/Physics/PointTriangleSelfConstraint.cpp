#include "PointTriangleSelfConstraint.h"

Physics::PointTriangleSelfConstraint::PointTriangleSelfConstraint(std::vector<Physics::ClothNode *> points, float stiffness, bool type, float h)
{
	m_points = points;
	m_stiffness = stiffness;
	m_type = type;
	m_h = h;
}

void Physics::PointTriangleSelfConstraint::Init()
{
}

float Physics::PointTriangleSelfConstraint::ComputeConstraint()
{
	return 15;
}

glm::vec3 Physics::PointTriangleSelfConstraint::ComputeDerivative(size_t index)
{
	return glm::vec3(-151);
}

float Physics::PointTriangleSelfConstraint::ComputeScalingFactor()
{
	return -151;
}

void Physics::PointTriangleSelfConstraint::Solve(int iterationCount)
{

	glm::vec3 q = m_points[0]->m_projection;
	glm::vec3 p1 = m_points[1]->m_projection;
	glm::vec3 p2 = m_points[2]->m_projection;
	glm::vec3 p3 = m_points[3]->m_projection;

	float wq = m_points[0]->m_invMass;
	float w1 = m_points[1]->m_invMass;
	float w2 = m_points[2]->m_invMass;
	float w3 = m_points[3]->m_invMass;

	glm::vec3 v1, v2;


	float dot = glm::dot(glm::normalize(q), glm::normalize(glm::cross(p2 - p1, p3 - p1)));

	if (dot >= 0)
	{
		v1 = p2 - p1;
		v2 = p3 - p1;
	}
	else
	{
		v1 = p3 - p1;
		v2 = p2 - p1;
	}

	glm::vec3 qp1 = q - p1;

	float constraintValue = glm::dot(qp1, glm::normalize(glm::cross(v1, v2))) - m_h;

	if (constraintValue >= 0)
		return;

	glm::vec3 derivQ, derivP1, derivP2, derivP3;
	glm::vec3 n;

	n = glm::normalize(glm::cross(v1, v2));

	// n aici
	derivQ = n;


	glm::mat3 p2m(glm::vec3(0, p2.z, -p2.y), glm::vec3(-p2.z, 0, p2.x), glm::vec3(p2.y, -p2.x, 0));
	glm::mat3 p3m(glm::vec3(0, p3.z, -p3.y), glm::vec3(-p3.z, 0, p3.x), glm::vec3(p3.y, -p3.x, 0));

	if (dot >= 0)
	{
		//derivP1 = -n + qp1 * (glm::cross(v1, minusOneVec) + glm::cross(minusOneVec, v2)) * term;

		derivP2 = q * (-p3m + glm::dot(n, glm::cross(n, p3))) / glm::length(glm::cross(p2, p3));
		derivP3 = -q * (-p2m + glm::dot(n, glm::cross(n, p2))) / glm::length(glm::cross(p2, p3));
	}
	else
	{

		//derivP1 = -n + qp1 * (glm::cross(minusOneVec, v2) + glm::cross(v1, minusOneVec)) * term;
		derivP2 = q * (-p2m + glm::dot(n, glm::cross(n, p2))) / glm::length(glm::cross(p3, p2));
		derivP3 = -q * (-p3m + glm::dot(n, glm::cross(n, p3))) / glm::length(glm::cross(p3, p2));
	}

	derivP1 = -derivQ - derivP2 - derivP3;

	float lenDQ = glm::length(derivQ);
	float lenDP1 = glm::length(derivP1);
	float lenDP2 = glm::length(derivP2);
	float lenDP3 = glm::length(derivP3);

	float kp = 1 - std::powf(1 - m_stiffness, 1.f / iterationCount);

	float scalingFactor = kp * constraintValue / (wq * lenDQ * lenDQ + w1 * lenDP1 * lenDP1 + w2 * lenDP2 * lenDP2 + w3 * lenDP3 * lenDP3);

	glm::vec3 correction1 = -scalingFactor * wq * derivQ;
	glm::vec3 correction2 = -scalingFactor * w1 * derivP1;
	glm::vec3 correction3 = -scalingFactor * w2 * derivP2;
	glm::vec3 correction4 = -scalingFactor * w3 * derivP3;

	if (glm::length(correction1) > EPS)
		m_points[0]->m_projection += correction1;
	if (glm::length(correction2) > EPS)
		m_points[1]->m_projection += correction2;
	if (glm::length(correction3) > EPS)
		m_points[2]->m_projection += correction3;
	if (glm::length(correction4) > EPS)
		m_points[3]->m_projection += correction4;
}


//x21.x * v31.y * x41.z * t + x21.x * v31.y * v41.z * t ^ 2 + x31.y * x21.x * x41.z + x31.y * x21.x * v41.z * t + v21.x * v31.y * x41.z * t ^ 2 + v21.x * v31.y * v41.z * t ^ 3 + x31.y * v21.x * x41.z * t + x31.y * v21.x * v41.z * t ^ 2
// f g o x + f g p x^2 + f h o x^2 + f h p x^3 + e g o + e g p x + e h o x + e h p x^2

//(q + r * t) * ((a + b * t) * (i + j * t) - (c + d * t) * (g + h * t)) - (o + p * t) * ((a + b * t) * (k + l * t) - (e + f * t) * (g + h * t)) + (m + n * t) * ((c + d * t) * (k + l * t) - (e + f * t) * (i + j * t)) = 0


/*a * j * q * t + a * j * r * t ^ 2 + a * q * z + a * r * t * z + b * j * q * t ^ 2 + b * j * r * t ^ 3 + b * q * t * z + b * r * t ^ 2 * z - c * g * q - c * g * r * t - c * h * q * t - c * h * r * t ^ 2 - d * g * q * t - d * g * r * t ^ 2 - d * h * q * t ^ 2 - d * h * r * t ^ 3 - a * k * o + a * k * p * t + a * l * o * t + a * l * p *t ^ 2 + b * k * o * t + b * k * p * t ^ 2 + b * l * o * t ^ 2 + b * l * p * t ^ 3 - f * g * o * t - f * g * p * t ^ 2 - f * h * o * t ^ 2 - f * h * p * t ^ 3 - e * g * o - e * g * p * t - e * h * o * t - e * h * p * t ^ 2 + c * k * m + c * k * n * t + c * l * m * t + c * l * n * t ^ 2 + d * k * m * t + d * k * n * t ^ 2 + d * l * m * t ^ 2 + d * l * n * t ^ 3 - f * j * m * t ^ 2 - f * j * n * t ^ 3 - f * m * t * z - f * n * t ^ 2 * z - e * j * m * t - e * j * n * t ^ 2 - e * m * z - e * n * t * z*/

// t3
// b * j * r * t ^ 3 + 
// - d * h * r * t ^ 3
// b * l * p * t ^ 3 
// - f * h * p * t ^ 3 
// d * l * n * t ^ 3 
// - f * j * n * t ^ 3 


//t^3 ( v21.x * v31.y * v41.z - v21.y * v31.x * v41.z + v21.x * v31.z * v41.y - v21.z * v31.x * v41.y + v21.y * v31.z * v41.x - v21.z * v31.y * v41.x) + 
// t2
//t ^ 2 * (x21.x * v31.y * v41.z + v21.x * v31.y * x41.z + v21.x * v41.z * x31.y - x21.y * v31.x * v41.z - v21.y * x31.x * v41.z - v21.y * v31.x * x41.z + x21.x * v31.z * v41.y + v21.x * x31.z * v41.y + v21.x * v31.z * x41.y - v21.z * x31.x * v41.y - v21.z * v31.x * x41.y - x21.z * v31.x * v41.y + x21.y * v31.z * v41.x + v21.y * x31.z * v41.x + v21.y * v31.z * x41.x - v21.z * v31.y * x41.x - v21.z * v41.x * x31.y - x21.z * v31.y * v41.x)
//t
// t (x21.x * v31.y * x41.z - x21.y * x31.x * v41.z - x21.y * v31.x * x41.z - v21.y * x31.x * x41.z + x21.x * x31.z * v41.y + x21.x * v31.z * x41.y + v21.x * x31.z * x41.y - v21.z * x31.x * x41.y - x21.z * x31.x * v41.y - x21.z * v31.x * x41.y + v21.x * x41.z * x31.y + x21.y * x31.z * v41.x + x21.y * v31.z * x41.x + v21.y * x31.z * x41.x - v21.z * x41.x * x31.y - x21.z * v31.y * x41.x - x21.z * v41.x * x31.y + x21.x * v41.z * x31.y)
// liber
// x21.x * x41.z * x31.y - x21.y * x31.x * x41.z - x21.x * x31.z * x41.y - x21.z * x31.x * x41.y + x21.y * x31.z * x41.x - x21.z * x41.x * x31.y


// a * j * r * t ^ 2 +
// b * j * q * t ^ 2 +
// b * r * t ^ 2 * z 
// - c * h * r * t ^ 2 
// - d * g * r * t ^ 2 
// - d * h * q * t ^ 2
// a * l * p * t ^ 2 +
// b * k * p * t ^ 2 +
// b * l * o * t ^ 2 +
// - f * g * p * t ^ 2 
// - f * h * o * t ^ 2 
// - e * h * p * t ^ 2 +
// c * l * n * t ^ 2 +
// d * k * n * t ^ 2 +
// d * l * m * t ^ 2 +
// - f * j * m * t ^ 2 
// - f * n * t ^ 2 * z 
// - e * j * n * t ^ 2 
// 
// 
// a * j * q * t +
// - c * g * r * t 
// - c * h * q * t 
// - d * g * q * t
// a * k * p * t +
// a * l * o * t +
// b * k * o * t +
// - f * g * o * t 
// - e * g * p * t 
// - e * h * o * t 
// b * q * t * z + 
// c * k * n * t + 
// c * l * m * t +
// d * k * m * t +
// - f * m * t * z 
// - e * j * m * t 
// - e * n * t * z
// a * r * t * z + 
// 
// a * q * z +
// - c * g * q 
// - a * k * o +
// - e * g * o 
// c * k * m +
// - e * m * z 
// 
// (m + n * t) * ((c + d * t) * (k + l * t) - (e + f * t) * (i + j * t))
// 
// 
// t ^ 3 * (bjr + )


// 	glm::vec3 x21 = t.m_triangle->m_nodes[0]->m_pos - p.m_node->m_pos;
// 	glm::vec3 x31 = t.m_triangle->m_nodes[1]->m_pos - p.m_node->m_pos;
// 	glm::vec3 x41 = t.m_triangle->m_nodes[2]->m_pos - p.m_node->m_pos;
// 
// 	glm::vec3 v21 = t.m_triangle->m_nodes[0]->m_vel - p.m_node->m_vel;
// 	glm::vec3 v31 = t.m_triangle->m_nodes[1]->m_vel - p.m_node->m_vel;
// 	glm::vec3 v41 = t.m_triangle->m_nodes[2]->m_vel - p.m_node->m_vel;
// 
// 	float a = v21.x * v31.y * v41.z - v21.y * v31.x * v41.z + v21.x * v31.z * v41.y - v21.z * v31.x * v41.y + v21.y * v31.z * v41.x - v21.z * v31.y * v41.x;
// 	float b = x21.x * v31.y * v41.z + v21.x * v31.y * x41.z + v21.x * v41.z * x31.y - x21.y * v31.x * v41.z - v21.y * x31.x * v41.z - v21.y * v31.x * x41.z + x21.x * v31.z * v41.y + v21.x * x31.z * v41.y + v21.x * v31.z * x41.y - v21.z * x31.x * v41.y - v21.z * v31.x * x41.y - x21.z * v31.x * v41.y + x21.y * v31.z * v41.x + v21.y * x31.z * v41.x + v21.y * v31.z * x41.x - v21.z * v31.y * x41.x - v21.z * v41.x * x31.y - x21.z * v31.y * v41.x;
// 	float c = x21.x * v31.y * x41.z - x21.y * x31.x * v41.z - x21.y * v31.x * x41.z - v21.y * x31.x * x41.z + x21.x * x31.z * v41.y + x21.x * v31.z * x41.y + v21.x * x31.z * x41.y - v21.z * x31.x * x41.y - x21.z * x31.x * v41.y - x21.z * v31.x * x41.y + v21.x * x41.z * x31.y + x21.y * x31.z * v41.x + x21.y * v31.z * x41.x + v21.y * x31.z * x41.x - v21.z * x41.x * x31.y - x21.z * v31.y * x41.x - x21.z * v41.x * x31.y + x21.x * v41.z * x31.y;
// 	float d = x21.x * x41.z * x31.y - x21.y * x31.x * x41.z - x21.x * x31.z * x41.y - x21.z * x31.x * x41.y + x21.y * x31.z * x41.x - x21.z * x41.x * x31.y;