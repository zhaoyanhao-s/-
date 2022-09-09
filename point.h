#pragma once
#include <iostream>

struct Point {
    Point() = default;
    Point(float x, float y, float z,Point* next = nullptr) : m_x(x), m_y(y), m_z(z), m_next(next){};
    float m_x{ 0.0f };
    float m_y{ 0.0f };
    float m_z{ 0.0f };
    Point* m_next{ nullptr };


    float operator[](int i);
};


//std::ostream& operator<<(std::ostream& os, const Point& pt) {
//    os << "{" << pt.m_x << ", " << pt.m_y << ", " << pt.m_z << "}";
//    return os;
//}
