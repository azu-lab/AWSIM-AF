#version 330 core

layout(location = 0) in vec3 aPos;   // 顶点坐标
layout(location = 1) in vec2 aTexCoord; // 纹理坐标

out vec2 TexCoords; // 输出的纹理坐标

void main()
{
    gl_Position = vec4(aPos, 1.0);
    TexCoords = aTexCoord; // 将纹理坐标传递给片段着色器
}

