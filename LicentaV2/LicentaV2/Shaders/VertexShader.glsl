#version 450 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

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
out vec2 uv;

void main()
{
	//vec4[5] colors = { inColor, vec4(1, 0, 0, 0), vec4(0, 1, 0, 0), vec4(0, 0, 1, 0), vec4(0, 0.5f, 0, 0) };

	//fragColor = colors[collisionState];


	vec3 scaledInPos = inPosition * 100.f;

	vec3 L = normalize(lightPosition - scaledInPos);
	vec3 V = normalize(eyePosition - scaledInPos);

	float LdotN = clamp(dot(L, inNormal), 0, 1);

	float ambientLight = 0.5;
	float distance = length(L);

	float diffuseLight = LdotN * kd;// / (distance * distance);

	float specularLight = 0;

	if (LdotN >= 0)
	{
		vec3 R = -normalize(reflect(L, inNormal));
		specularLight = ks * pow(max(dot(V, R), 0), shininess);
	}

	float l1 = (ambientLight + diffuseLight) + specularLight;

	vec3 lPos = lightPosition;
	lPos.z = -lPos.z;

	L = normalize(lPos - scaledInPos);
	LdotN = clamp(dot(L, inNormal), 0, 1);
	diffuseLight = LdotN * kd;// / (distance * distance);

	specularLight = 0;

	if (LdotN >= 0)
	{
		vec3 R = -normalize(reflect(L, inNormal));
		specularLight = ks * pow(max(dot(V, R), 0), shininess);
	}	

	float l2 = (ambientLight + diffuseLight) + specularLight;

	light = l1 + l2;//(ambientLight + diffuseLight) + specularLight;

	fragColor = inColor;
	gl_Position = viewProjection * vec4(scaledInPos, 1);
	uv = inUV;
}