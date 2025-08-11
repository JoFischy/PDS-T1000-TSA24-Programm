#ifndef MOVEMENT_SYSTEM_H
#define MOVEMENT_SYSTEM_H

#include "point.h"
#include "path_system.h"
#include <vector>

enum class Direction {
    NORTH = 0,
    EAST = 90,
    SOUTH = 180,
    WEST = 270
};

// Abstract interface for movement commands
class MovementInterface {
public:
    virtual ~MovementInterface() = default;
    virtual void moveForward(int vehicleId, float distance) = 0;
    virtual void turnLeft(int vehicleId) = 0;
    virtual void turnRight(int vehicleId) = 0;
    virtual void setPosition(int vehicleId, const Point& position, Direction direction) = 0;
};

#endif