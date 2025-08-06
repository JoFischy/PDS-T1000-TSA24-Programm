
#ifndef POINT_H
#define POINT_H

enum class PointType {
    IDENTIFICATION,
    FRONT
};

struct Point {
    float x;
    float y;
    bool isDragging;
    PointType type;
    
    Point() : x(0.0f), y(0.0f), isDragging(false), type(PointType::IDENTIFICATION) {}
    Point(float x, float y, PointType t = PointType::IDENTIFICATION) : x(x), y(y), isDragging(false), type(t) {}
    
    // Calculate distance to another point
    float distanceTo(const Point& other) const;
    
    // Check if mouse is over this point
    bool isMouseOver(float mouseX, float mouseY, float radius = 10.0f) const;
};

#endif
