#pragma once
#include "point.h"
#include "movement_system.h"
#include <vector>

enum class VehicleState {
    IDLE,
    MOVING,
    WAITING,
    ARRIVED
};

class Auto {
private:
    static int nextId;
    
public:
    // Original members from the cpp implementation
    bool valid;
    Direction currentDirection;
    float speed;
    int id;
    int currentSegmentId;
    bool isMoving;
    bool isWaitingInQueue;
    Point position;
    Point targetPosition;
    
    // New members from the previous struct version
    VehicleState state;
    std::vector<int> currentPath;
    int currentSegmentIndex;
    int currentNodeId;
    int targetNodeId;
    float size;
    int vehicleId;
    
    // Constructors
    Auto();
    Auto(const Point& startPos, Direction dir);
    Auto(int vehicleId, const Point& startPos); // Constructor for vehicle controller
    
    // Methods expected by the cpp implementation
    void setPosition(const Point& pos);
    void setTargetPosition(const Point& target);
    void updatePosition(float deltaTime);
    void calculateDirection();
    
    // Getter methods for compatibility
    int getId() const { return id; }
    bool isValid() const { return valid; }
};