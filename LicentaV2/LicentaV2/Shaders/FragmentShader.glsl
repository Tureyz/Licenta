#version 450 core

layout(location = 0) out vec4 outColor;

in vec4 fragColor;
in float light;
 
void main(void)
{
	if (fragColor.x == 0 && fragColor.y == 0 && fragColor.z == 1)
	{
		outColor = fragColor;
	}
	else
	{
		outColor = vec4(fragColor.x * light, fragColor.y * light, fragColor.z * light, 1);
	}
}