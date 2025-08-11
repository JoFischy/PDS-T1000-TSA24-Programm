#pragma once
#include <cmath>

struct Point {
    float x, y;
    
    Point() : x(0), y(0) {}
    Point(float x_val, float y_val) : x(x_val), y(y_val) {}
    
    float distanceTo(const Point& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
    
    Point operator*(float scalar) const {
        return Point(x * scalar, y * scalar);
    }
    
    bool operator==(const Point& other) const {
        const float epsilon = 0.001f;
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }
    
    bool isMouseOver(float mouseX, float mouseY, float radius) const {
        float dx = x - mouseX;
        float dy = y - mouseY;
        return std::sqrt(dx * dx + dy * dy) <= radius;
    }
    
    Point normalize() const {
        float length = std::sqrt(x * x + y * y);
        if (length == 0) return Point(0, 0);
        return Point(x / length, y / length);
    }
    
    float length() const {
        return std::sqrt(x * x + y * y);
    }
};