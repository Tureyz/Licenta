#version 450 core

layout(location = 0) out vec4 outColor;

in vec3 fragPos;
in vec3 fragNormal;
in vec4 fragColor;
in vec2 fragUV;

uniform vec3 eyePosition;
uniform vec3 lightPosition;
uniform int shininess;
uniform float kd;
uniform float ks;

uniform sampler2D myTextureSampler;
 
void main(void)
{
	//if (fragColor.x == 0 && fragColor.y == 0 && fragColor.z == 1)
	//{
	//	outColor = fragColor;
	//}
	//else
	//{
	//	vec3 tex = texture(myTextureSampler, uv).rgb;

	//	//outColor = vec4(fragColor.x * light * tex.x, fragColor.y * light * tex.y, fragColor.z * light * tex.z, 1);
	//	outColor = vec4(fragColor.x * light , fragColor.y * light , fragColor.z * light, 1);
	//}


	vec3 lPos = lightPosition * 100.f;
	float ambientLight = 0.5f;
	vec3 L = normalize(lightPosition - fragPos);
	vec3 V = normalize(eyePosition - fragPos);

	float LdotN = clamp(dot(L, fragNormal), 0, 1);

	float diffuse = kd * LdotN;

	float specular = 0;

	
	vec3 R = -normalize(reflect(L, fragNormal));
	specular = ks * pow(max(0, dot(R, V)), shininess);

	//vec3 H = normalize(L + V);
	//specular = ks * pow(max(0, dot(H, fragNormal)), shininess);

	float l1 = ambientLight + diffuse + specular;

	

	lPos.z = -lPos.z;


	L = normalize(lPos - fragPos);
	V = normalize(eyePosition - fragPos);

	LdotN = clamp(dot(L, fragNormal), 0, 1);

	diffuse = kd * LdotN;

	specular = 0;


	R = -normalize(reflect(L, fragNormal));
	specular = ks * pow(max(0, dot(R, V)), shininess);

	//H = normalize(L + V);
	//specular = ks * pow(max(0, dot(H, fragNormal)), shininess);

	float l2 = ambientLight + diffuse + specular;


	float light = l1 + l2;
	
	
	vec3 tex = texture(myTextureSampler, fragUV).rgb;

	//outColor = vec4(fragColor.x * light, fragColor.y * light, fragColor.z * light, 1);
	outColor = vec4(fragColor.x * light * tex.x, fragColor.y * light * tex.y, fragColor.z * light * tex.z, 1);
}