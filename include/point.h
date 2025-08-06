#ifndef POINT_H
#define POINT_H

struct Point {
    float x;
    float y;
    
    Point() : x(0.0f), y(0.0f) {}
    Point(float x, float y) : x(x), y(y) {}
    
    // Calculate distance to another point
    float distanceTo(const Point& other) const;
    
    // Calculate angle to another point in degrees (0° = up, 90° = right)
    float angleTo(const Point& other) const;
};

#endif
