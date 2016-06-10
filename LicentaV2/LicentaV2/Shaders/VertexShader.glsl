#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

//uniform mat4 projectionMatrix, viewMatrix, modelMatrix;

layout(location = 2) uniform int collisionState;
layout(location = 3) uniform mat4 MVPMatrix;

out vec4 fragColor;

void main()
{
	vec4[5] colors = { inColor, vec4(1, 0, 0, 0), vec4(0, 1, 0, 0), vec4(0, 0, 1, 0), vec4(0, 0.5f, 0, 0) };
	fragColor = colors[collisionState];
	gl_Position = MVPMatrix * vec4(inPosition, 1);
}