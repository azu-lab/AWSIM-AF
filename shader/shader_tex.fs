#version 330 core

in vec2 TexCoords;  // 纹理坐标从顶点着色器传递过来
out vec4 FragColor; // 输出的片段颜色

uniform sampler2D texture1; // 纹理采样器

void main()
{
    FragColor = texture(texture1, TexCoords); // 从纹理采样器中获取颜色
}

