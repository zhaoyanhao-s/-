#include "point.h"

float Point::operator[](int i) {
    float b[3] = { m_x,m_y,m_z };
    return b[i];
}
