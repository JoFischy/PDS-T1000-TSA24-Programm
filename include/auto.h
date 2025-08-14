#pragma once
#include "point.h"
#include <vector>

enum class VehicleState {
    IDLE,
    MOVING,
    WAITING,
    ARRIVED
};

enum class Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

class Auto {
public:
    int vehicleId;
    Point position;
    Point targetPosition;
    VehicleState state;
    Direction currentDirection;
    bool isMoving;

    int currentNodeId;
    int targetNodeId;

    std::vector<int> currentPath;  // Pfad als Liste von Segment-IDs
    int currentSegmentIndex;

    float speed;
    float progress; // Fortschritt auf aktuellem Segment (0.0 - 1.0)

    static int nextId;

    Auto();
    Auto(int id, const Point& startPos);

    void setPosition(const Point& pos);
    void setTargetPosition(const Point& target);
    void updatePosition(float deltaTime);
    void calculateDirection();
    bool isAtTarget() const;
    void reset();
};