

#include "auto.h"
#include <cmath>

int Auto::nextId = 1;

Auto::Auto() : valid(false), direction(0.0f), id(0) {}

Auto::Auto(const Point& idPoint, const Point& fPoint) 
    : identificationPoint(idPoint), frontPoint(fPoint), valid(true), id(nextId++) {
    calculateCenterAndDirection();
}

void Auto::updatePoints(const Point& idPoint, const Point& fPoint) {
    identificationPoint = idPoint;
    frontPoint = fPoint;
    valid = true;
    calculateCenterAndDirection();
}

void Auto::calculateCenterAndDirection() {
    // Calculate center point
    center.x = (identificationPoint.x + frontPoint.x) / 2.0f;
    center.y = (identificationPoint.y + frontPoint.y) / 2.0f;
    
    // Calculate direction (vector from identification point to front point)
    float dx = frontPoint.x - identificationPoint.x;
    float dy = frontPoint.y - identificationPoint.y;
    direction = atan2f(dy, dx) * 180.0f / M_PI;
    if (direction < 0) {
        direction += 360.0f;
    }
}

