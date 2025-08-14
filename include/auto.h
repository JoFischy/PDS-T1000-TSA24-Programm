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

    int waitingForSegment;  // Segment ID we're waiting for
    bool isAtWaitingNode;   // True if currently at a waiting node

    static int nextId;

    Auto();
    Auto(int id, const Point& startPos)
        : vehicleId(id), position(startPos), targetPosition(startPos),
          state(VehicleState::IDLE), currentDirection(Direction::NORTH),
          isMoving(false), currentNodeId(-1), targetNodeId(-1),
          currentSegmentIndex(0), speed(100.0f), progress(0.0f), 
          waitingForSegment(-1), isAtWaitingNode(false) {}

    void setPosition(const Point& pos);
    void setTargetPosition(const Point& target);
    void updatePosition(float deltaTime);
    void calculateDirection();
    bool isAtTarget() const;
    void reset();
};