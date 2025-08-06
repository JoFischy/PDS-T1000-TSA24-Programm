#include "auto.h"
#include <cmath>

const float Auto::VEHICLE_LENGTH = 160.0f;
const float Auto::TOLERANCE = 20.0f;

Auto::Auto(int autoId) : id(autoId), isValid(false), direction(0.0f), 
    lastKnownDirection(0.0f), frontPointValid(false), rearPointValid(false), 
    timeSinceLastUpdate(0.0f) {
}

bool Auto::updatePoints(const Point& newFront, const Point& newRear) {
    // Check if both points are valid (not detection failures)
    bool frontValid = (newFront.x > 0 && newFront.y > 0);
    bool rearValid = (newRear.x > 0 && newRear.y > 0);
    
    if (!frontValid && !rearValid) {
        return false; // No valid points
    }
    
    if (frontValid && rearValid) {
        float distance = newFront.distanceTo(newRear);
        
        // Check if the distance is approximately correct for a vehicle
        if (std::abs(distance - VEHICLE_LENGTH) > TOLERANCE * 2) {
            return false;
        }
        
        // If this auto already has points, check if new points are close enough
        if (isValid && frontPointValid && rearPointValid) {
            float frontDist = frontPoint.distanceTo(newFront);
            float rearDist = rearPoint.distanceTo(newRear);
            
            // If points are too far from previous position, reject update
            if (frontDist > TOLERANCE * 4 || rearDist > TOLERANCE * 4) {
                return false;
            }
        }
        
        frontPoint = newFront;
        rearPoint = newRear;
        frontPointValid = true;
        rearPointValid = true;
    } else if (frontValid) {
        return updateFrontPoint(newFront);
    } else if (rearValid) {
        return updateRearPoint(newRear);
    }
    
    lastKnownPosition = position;
    lastKnownDirection = direction;
    calculatePositionAndDirection();
    isValid = true;
    timeSinceLastUpdate = 0.0f;
    return true;
}

bool Auto::updateFrontPoint(const Point& newFront) {
    if (newFront.x <= 0 || newFront.y <= 0) {
        frontPointValid = false;
        return false;
    }
    
    // If we have a valid rear point, check distance
    if (rearPointValid) {
        float distance = newFront.distanceTo(rearPoint);
        if (std::abs(distance - VEHICLE_LENGTH) > TOLERANCE * 2) {
            return false;
        }
    }
    
    // If we had a valid front point before, check if new point is close enough
    if (frontPointValid && isValid) {
        float frontDist = frontPoint.distanceTo(newFront);
        if (frontDist > TOLERANCE * 4) {
            return false;
        }
    }
    
    frontPoint = newFront;
    frontPointValid = true;
    
    // If we only have front point, estimate rear point from last known direction
    if (!rearPointValid && isValid) {
        float dirRad = lastKnownDirection * M_PI / 180.0f;
        rearPoint.x = frontPoint.x - sinf(dirRad) * VEHICLE_LENGTH;
        rearPoint.y = frontPoint.y + cosf(dirRad) * VEHICLE_LENGTH;
        rearPointValid = true;
    }
    
    lastKnownPosition = position;
    lastKnownDirection = direction;
    calculatePositionAndDirection();
    timeSinceLastUpdate = 0.0f;
    return true;
}

bool Auto::updateRearPoint(const Point& newRear) {
    if (newRear.x <= 0 || newRear.y <= 0) {
        rearPointValid = false;
        return false;
    }
    
    // If we have a valid front point, check distance
    if (frontPointValid) {
        float distance = frontPoint.distanceTo(newRear);
        if (std::abs(distance - VEHICLE_LENGTH) > TOLERANCE * 2) {
            return false;
        }
    }
    
    // If we had a valid rear point before, check if new point is close enough
    if (rearPointValid && isValid) {
        float rearDist = rearPoint.distanceTo(newRear);
        if (rearDist > TOLERANCE * 4) {
            return false;
        }
    }
    
    rearPoint = newRear;
    rearPointValid = true;
    
    // If we only have rear point, estimate front point from last known direction
    if (!frontPointValid && isValid) {
        float dirRad = lastKnownDirection * M_PI / 180.0f;
        frontPoint.x = rearPoint.x + sinf(dirRad) * VEHICLE_LENGTH;
        frontPoint.y = rearPoint.y - cosf(dirRad) * VEHICLE_LENGTH;
        frontPointValid = true;
    }
    
    lastKnownPosition = position;
    lastKnownDirection = direction;
    calculatePositionAndDirection();
    timeSinceLastUpdate = 0.0f;
    return true;
}

void Auto::updateTime(float deltaTime) {
    timeSinceLastUpdate += deltaTime;
    
    // If no points detected for a while, predict position
    if (timeSinceLastUpdate > 0.1f && isValid) {
        predictPosition(deltaTime);
    }
}

void Auto::calculatePositionAndDirection() {
    if (frontPointValid && rearPointValid) {
        // Calculate midpoint
        position.x = (frontPoint.x + rearPoint.x) / 2.0f;
        position.y = (frontPoint.y + rearPoint.y) / 2.0f;
        
        // Calculate direction from rear to front (0° = up, 90° = right)
        float dx = frontPoint.x - rearPoint.x;
        float dy = frontPoint.y - rearPoint.y;
        
        // atan2 gives angle where 0° is right, we want 0° to be up
        float angle = atan2f(dx, -dy) * 180.0f / M_PI;
        
        // Normalize to 0-359 range
        if (angle < 0) {
            angle += 360.0f;
        }
        
        direction = angle;
    } else if (frontPointValid) {
        // Only front point available, use last known direction
        float dirRad = lastKnownDirection * M_PI / 180.0f;
        position.x = frontPoint.x - sinf(dirRad) * (VEHICLE_LENGTH / 2.0f);
        position.y = frontPoint.y + cosf(dirRad) * (VEHICLE_LENGTH / 2.0f);
        // Keep last known direction
    } else if (rearPointValid) {
        // Only rear point available, use last known direction
        float dirRad = lastKnownDirection * M_PI / 180.0f;
        position.x = rearPoint.x + sinf(dirRad) * (VEHICLE_LENGTH / 2.0f);
        position.y = rearPoint.y - cosf(dirRad) * (VEHICLE_LENGTH / 2.0f);
        // Keep last known direction
    }
    // If no points are valid, position and direction remain unchanged
}

void Auto::predictPosition(float deltaTime) {
    if (!isValid) return;
    
    // Simple prediction: continue in last known direction at estimated speed
    float estimatedSpeed = 40.0f; // pixels per second
    float dirRad = lastKnownDirection * M_PI / 180.0f;
    
    position.x += sinf(dirRad) * estimatedSpeed * deltaTime;
    position.y -= cosf(dirRad) * estimatedSpeed * deltaTime;
    
    // Update estimated front and rear points
    frontPoint.x = position.x + sinf(dirRad) * (VEHICLE_LENGTH / 2.0f);
    frontPoint.y = position.y - cosf(dirRad) * (VEHICLE_LENGTH / 2.0f);
    
    rearPoint.x = position.x - sinf(dirRad) * (VEHICLE_LENGTH / 2.0f);
    rearPoint.y = position.y + cosf(dirRad) * (VEHICLE_LENGTH / 2.0f);
}

bool Auto::couldBelongToAuto(const Point& front, const Point& rear) const {
    if (!isValid) {
        return true; // Any points can belong to an uninitialized auto
    }
    
    float frontDist = frontPoint.distanceTo(front);
    float rearDist = rearPoint.distanceTo(rear);
    
    // Check if points are within tolerance of current position
    return (frontDist <= TOLERANCE * 2 && rearDist <= TOLERANCE * 2);
}

float Point::distanceTo(const Point& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrtf(dx * dx + dy * dy);
}

float Point::angleTo(const Point& other) const {
    float dx = other.x - x;
    float dy = other.y - y;
    float angle = atan2f(dx, -dy) * 180.0f / M_PI;
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle;
}
