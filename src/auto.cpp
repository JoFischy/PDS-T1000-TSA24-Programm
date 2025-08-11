#include "auto.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int Auto::nextId = 1;

Auto::Auto() : valid(false), currentDirection(Direction::NORTH), speed(2.0f), id(0), 
               currentSegmentId(-1), isMoving(false), isWaitingInQueue(false) {}

Auto::Auto(const Point& startPos, Direction dir) 
    : position(startPos), targetPosition(startPos), currentDirection(dir), 
      speed(2.0f), valid(true), id(nextId++), currentSegmentId(-1), 
      isMoving(false), isWaitingInQueue(false) {}

Auto::Auto(int id, const Point& startPos) 
    : vehicleId(id), position(startPos), currentNodeId(-1), targetNodeId(-1), pendingTargetNodeId(-1),
      currentSegmentIndex(0), speed(50.0f), state(VehicleState::IDLE), isWaitingAtSafetyStop(false) {}

void Auto::setPosition(const Point& pos) {
    position = pos;
}

void Auto::setTargetPosition(const Point& target) {
    targetPosition = target;
    if (position.distanceTo(target) > 0.1f) {
        calculateDirection();
    }
}

void Auto::updatePosition(float deltaTime) {
    if (!isMoving || position.distanceTo(targetPosition) < 1.0f) {
        return;
    }

    // Move towards target position
    Point direction = targetPosition - position;
    float distance = position.distanceTo(targetPosition);

    if (distance > 0) {
        Point normalizedDir = direction * (1.0f / distance);
        float moveDistance = speed * deltaTime * 60.0f; // Assuming 60 FPS

        if (moveDistance >= distance) {
            position = targetPosition;
        } else {
            position = position + normalizedDir * moveDistance;
        }
    }
}

void Auto::calculateDirection() {
    Point dir = targetPosition - position;
    float angle = atan2f(dir.y, dir.x);

    // Convert to our Direction enum (0=North, 90=East, etc.)
    float degrees = angle * 180.0f / M_PI;
    if (degrees < 0) degrees += 360.0f;

    // Snap to nearest 90-degree direction
    if (degrees >= 315.0f || degrees < 45.0f) {
        currentDirection = Direction::EAST;
    } else if (degrees >= 45.0f && degrees < 135.0f) {
        currentDirection = Direction::SOUTH;
    } else if (degrees >= 135.0f && degrees < 225.0f) {
        currentDirection = Direction::WEST;
    } else {
        currentDirection = Direction::NORTH;
    }
}