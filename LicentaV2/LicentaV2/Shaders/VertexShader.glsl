#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec3 inNormal;

//uniform mat4 projectionMatrix, viewMatrix, modelMatrix;

//layout(location = 2) uniform int collisionState;
layout(location = 3) uniform mat4 viewProjection;

uniform vec3 eyePosition;
uniform vec3 lightPosition;
uniform int shininess;
uniform float kd;
uniform float ks;

out vec4 fragColor;
out float light;

void main()
{
	//vec4[5] colors = { inColor, vec4(1, 0, 0, 0), vec4(0, 1, 0, 0), vec4(0, 0, 1, 0), vec4(0, 0.5f, 0, 0) };

	//fragColor = colors[collisionState];

	vec3 L = normalize(lightPosition - inPosition);
	vec3 V = normalize(eyePosition - inPosition);

	float ambientLight = 0.6;
	float LdotN = clamp(dot(-L, inNormal), 0, 1);
	float diffuseLight = LdotN * kd;
	float specularLight = 0;

	if (LdotN > 0.0)
	{
		vec3 R = -normalize(reflect(L, inNormal));
		specularLight = ks * pow(max(dot(V, R), 0), shininess);
	}

	light = (ambientLight + diffuseLight) + specularLight;

	fragColor = inColor;
	gl_Position = viewProjection * vec4(inPosition, 1);
}