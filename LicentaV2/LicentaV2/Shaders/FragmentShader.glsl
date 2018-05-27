#version 450 core

layout(location = 0) out vec4 outColor;

in vec4 fragColor;
in float light;
in vec2 uv;

uniform sampler2D myTextureSampler;
 
void main(void)
{
	if (fragColor.x == 0 && fragColor.y == 0 && fragColor.z == 1)
	{
		outColor = fragColor;
	}
	else
	{
		vec3 tex = texture(myTextureSampler, uv).rgb;

		//outColor = vec4(fragColor.x * light * tex.x, fragColor.y * light * tex.y, fragColor.z * light * tex.z, 1);
		outColor = vec4(fragColor.x * light , fragColor.y * light , fragColor.z * light, 1);
	}
}