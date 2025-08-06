
#include "vehicle_controller.h"
#include <cmath>

VehicleController::VehicleController() {
    createSamplePath();
}

Direction VehicleController::calculateDirection(const Point& from, const Point& to) const {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    
    // Determine primary direction based on larger component
    if (std::abs(dx) > std::abs(dy)) {
        return dx > 0 ? Direction::EAST : Direction::WEST;
    } else {
        return dy > 0 ? Direction::SOUTH : Direction::NORTH;
    }
}

Point VehicleController::moveTowardsTarget(const Point& current, const Point& target, float speed) const {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance <= speed) {
        return target; // Reached target
    }
    
    // Normalize and apply speed
    float ratio = speed / distance;
    Point newPos;
    newPos.x = current.x + dx * ratio;
    newPos.y = current.y + dy * ratio;
    return newPos;
}

void VehicleController::moveForward(int vehicleId, float distance) {
    auto it = vehicleStates.find(vehicleId);
    if (it == vehicleStates.end()) return;
    
    VehicleState& state = it->second;
    float radians = static_cast<float>(state.currentDirection) * M_PI / 180.0f;
    
    state.currentPosition.x += std::cos(radians) * distance;
    state.currentPosition.y += std::sin(radians) * distance;
}

void VehicleController::turnLeft(int vehicleId) {
    auto it = vehicleStates.find(vehicleId);
    if (it == vehicleStates.end()) return;
    
    VehicleState& state = it->second;
    int newDir = (static_cast<int>(state.currentDirection) - 90 + 360) % 360;
    state.currentDirection = static_cast<Direction>(newDir);
}

void VehicleController::turnRight(int vehicleId) {
    auto it = vehicleStates.find(vehicleId);
    if (it == vehicleStates.end()) return;
    
    VehicleState& state = it->second;
    int newDir = (static_cast<int>(state.currentDirection) + 90) % 360;
    state.currentDirection = static_cast<Direction>(newDir);
}

void VehicleController::setPosition(int vehicleId, const Point& position, Direction direction) {
    vehicleStates[vehicleId].currentPosition = position;
    vehicleStates[vehicleId].currentDirection = direction;
}

void VehicleController::addVehicle(const Auto& vehicle) {
    if (!vehicle.isValid()) return;
    
    VehicleState state;
    state.currentPosition = vehicle.getCenter();
    state.currentDirection = Direction::NORTH; // Default direction
    state.currentPathIndex = 0;
    state.isMoving = true;
    
    vehicleStates[vehicle.getId()] = state;
}

void VehicleController::removeVehicle(int vehicleId) {
    vehicleStates.erase(vehicleId);
}

void VehicleController::updateVehicle(int vehicleId, Auto& vehicle) {
    auto it = vehicleStates.find(vehicleId);
    if (it == vehicleStates.end() || !vehicle.isValid()) return;
    
    VehicleState& state = it->second;
    
    if (!state.isMoving || pathSystem.getPathSize() == 0) return;
    
    // Get current target node
    if (state.currentPathIndex >= pathSystem.getPathSize()) {
        state.currentPathIndex = 0; // Loop back to start
    }
    
    PathNode targetNode = pathSystem.getPath()[state.currentPathIndex];
    
    // Move towards target
    Point newPos = moveTowardsTarget(state.currentPosition, targetNode.position, state.speed);
    
    // Check if reached target node
    float distanceToTarget = state.currentPosition.distanceTo(targetNode.position);
    if (distanceToTarget < state.speed * 2.0f) {
        state.currentPosition = targetNode.position;
        state.currentDirection = targetNode.direction;
        state.currentPathIndex = (state.currentPathIndex + 1) % pathSystem.getPathSize();
    } else {
        state.currentPosition = newPos;
        // Smooth direction transition towards target
        state.currentDirection = calculateDirection(state.currentPosition, targetNode.position);
    }
    
    // Update the Auto object with new position
    // Calculate front point based on direction
    float dirRadians = static_cast<float>(state.currentDirection) * M_PI / 180.0f;
    Point idPoint = state.currentPosition;
    Point frontPoint;
    frontPoint.x = idPoint.x + std::cos(dirRadians) * 30.0f; // 30 pixel offset
    frontPoint.y = idPoint.y + std::sin(dirRadians) * 30.0f;
    
    vehicle.updatePoints(idPoint, frontPoint);
}

void VehicleController::updateAllVehicles(std::vector<Auto>& vehicles) {
    for (Auto& vehicle : vehicles) {
        if (vehicle.isValid()) {
            updateVehicle(vehicle.getId(), vehicle);
        }
    }
}

void VehicleController::createSamplePath() {
    pathSystem.clearPath();
    
    // Create a rectangular path
    float centerX = 960.0f;
    float centerY = 600.0f;
    float width = 300.0f;
    float height = 200.0f;
    
    // Top edge (going east)
    pathSystem.addNode(centerX - width/2, centerY - height/2, Direction::EAST);
    pathSystem.addNode(centerX + width/2, centerY - height/2, Direction::SOUTH);
    
    // Right edge (going south)
    pathSystem.addNode(centerX + width/2, centerY + height/2, Direction::WEST);
    
    // Bottom edge (going west)
    pathSystem.addNode(centerX - width/2, centerY + height/2, Direction::NORTH);
    
    // Complete the loop
    pathSystem.addNode(centerX - width/2, centerY - height/2, Direction::EAST);
}

void VehicleController::setVehicleSpeed(int vehicleId, float speed) {
    auto it = vehicleStates.find(vehicleId);
    if (it != vehicleStates.end()) {
        it->second.speed = speed;
    }
}

bool VehicleController::isVehicleMoving(int vehicleId) const {
    auto it = vehicleStates.find(vehicleId);
    if (it != vehicleStates.end()) {
        return it->second.isMoving;
    }
    return false;
}

void VehicleController::stopVehicle(int vehicleId) {
    auto it = vehicleStates.find(vehicleId);
    if (it != vehicleStates.end()) {
        it->second.isMoving = false;
    }
}

void VehicleController::startVehicle(int vehicleId) {
    auto it = vehicleStates.find(vehicleId);
    if (it != vehicleStates.end()) {
        it->second.isMoving = true;
    }
}
