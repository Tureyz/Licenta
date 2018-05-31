#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

//uniform mat4 projectionMatrix, viewMatrix, modelMatrix;

//layout(location = 2) uniform int collisionState;
layout(location = 3) uniform mat4 viewProjection;



out vec3 fragPos;
out vec3 fragNormal;
out vec4 fragColor;
out vec2 fragUV;

void main()
{

	fragPos = inPosition * 100.f;
	fragColor = inColor;
	fragNormal = inNormal;
	fragUV = inUV;

	gl_Position = viewProjection * vec4(fragPos, 1);
}