#version 330 core

//片段着色器
//out vec4 FragColor;
//in vec2 TexCord;
//uniform sampler2D textureWall;
//uniform sampler2D textureSmile;
//uniform sampler2D textureSmall;
//uniform float ratio;
//void main(){
//    FragColor = mix(texture(textureWall,TexCord),
//                    texture(textureSmile,TexCord),ratio);
//}
out vec4 FragColor;
uniform vec4 color;
void main(){
    //FragColor = vec4(0.f, 255.f, 255.f, 1.0f);
    FragColor = color;
}
