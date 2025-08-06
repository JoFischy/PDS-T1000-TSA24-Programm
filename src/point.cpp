
#include "point.h"
#include <cmath>

float Point::distanceTo(const Point& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrtf(dx * dx + dy * dy);
}

bool Point::isMouseOver(float mouseX, float mouseY, float radius) const {
    float dx = x - mouseX;
    float dy = y - mouseY;
    return sqrtf(dx * dx + dy * dy) <= radius;
}
