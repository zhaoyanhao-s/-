#pragma once
#include "point.h"

//既是一个矩形，也是一个OOB计算单元
struct Cuboid  {
    float m_halflength{ 0.0f };
    float m_halfwidth{ 0.0f };
    float m_halfheight{ 0.0f };
    float m_R2{ 0.0f};
    //用于OOB计算的三个旋转轴(就是垂直于对应面的方向角，例如Axis就是垂直x面的方向角)
    //对于八叉树中的节点，都是沿坐标轴的方向
    Point m_axeX{ 1.0f,0.0f,0.0f,nullptr};
    Point m_axeY{ 0.0f,1.0f,0.0f,nullptr};
    Point m_axeZ{ 0.0f,0.0f,1.0f,nullptr};
    //矩形的中心点
    Point m_center{ 0.0f,0.0f,0.0f };

    Cuboid() {};
    Cuboid(const float& halflength, const float& halfwidth, const float&halfheight, const Point& center) :
        m_halflength(halflength),
        m_halfwidth(halfwidth),
        m_halfheight(halfheight),
        m_center(center) {
    };

    void setAxe(const float& roation);

    float operator[](int i);
};
