#include "cuboid.h"
#include <cmath>

//给出机器人的朝向，并设置它的旋转轴，根据题意，z轴始终沿z轴方向，不需要改变
void Cuboid::setAxe(const float& rotation) {
    m_axeX.m_x = m_axeY.m_y = cos(rotation);
    m_axeX.m_y = sin(rotation);
    m_axeY.m_x = -sin(rotation);
}

float Cuboid::operator[](int i) {
    float b[3] = { m_halflength,m_halfwidth,m_halfwidth };
    return b[i];
}
