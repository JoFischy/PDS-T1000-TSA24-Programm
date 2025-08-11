#include "vehicle_controller.h"
#include <algorithm>
#include <random>
#include <iostream>

VehicleController::VehicleController(PathSystem* pathSys, SegmentManager* segmentMgr)
    : pathSystem(pathSys), segmentManager(segmentMgr), nextVehicleId(0) {}

int VehicleController::addVehicle(const Point& startPosition) {
    Auto vehicle(nextVehicleId, startPosition);
    
    // Find nearest node as starting position
    int nearestNodeId = pathSystem->findNearestNode(startPosition);
    if (nearestNodeId != -1) {
        vehicle.currentNodeId = nearestNodeId;
        const PathNode* node = pathSystem->getNode(nearestNodeId);
        if (node) {
            vehicle.position = node->position;
        }
    }
    
    vehicles.push_back(vehicle);
    vehicleIdToIndex[nextVehicleId] = vehicles.size() - 1;
    
    return nextVehicleId++;
}

void VehicleController::spawnInitialVehicles() {
    if (!pathSystem || pathSystem->getNodeCount() < 4) {
        std::cerr << "Not enough nodes to spawn 4 vehicles" << std::endl;
        return;
    }
    
    const auto& nodes = pathSystem->getNodes();
    
    // Clear existing vehicles
    vehicles.clear();
    vehicleIdToIndex.clear();
    nextVehicleId = 0;
    
    // Spawn 4 vehicles at different nodes
    int nodeStep = std::max(1, static_cast<int>(nodes.size()) / 4);
    
    for (int i = 0; i < 4 && i * nodeStep < nodes.size(); i++) {
        const PathNode& node = nodes[i * nodeStep];
        addVehicle(node.position);
        std::cout << "Spawned vehicle " << (i + 1) << " at node " << node.nodeId 
                  << " (" << node.position.x << ", " << node.position.y << ")" << std::endl;
    }
    
    std::cout << "Spawned " << vehicles.size() << " initial vehicles" << std::endl;
    
    // Assign random targets to all vehicles
    assignRandomTargetsToAllVehicles();
}

void VehicleController::assignRandomTargetsToAllVehicles() {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;
    
    const auto& nodes = pathSystem->getNodes();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, nodes.size() - 1);
    
    for (auto& vehicle : vehicles) {
        // Find a different target node than current one
        int targetNodeId;
        do {
            int randomIndex = nodeDist(gen);
            targetNodeId = nodes[randomIndex].nodeId;
        } while (targetNodeId == vehicle.currentNodeId && nodes.size() > 1);
        
        setVehicleTarget(vehicle.vehicleId, targetNodeId);
        std::cout << "Vehicle " << vehicle.vehicleId << " assigned target node " << targetNodeId << std::endl;
    }
}

void VehicleController::removeVehicle(int vehicleId) {
    auto it = vehicleIdToIndex.find(vehicleId);
    if (it == vehicleIdToIndex.end()) return;
    
    size_t index = it->second;
    
    // Release any occupied segments
    segmentManager->removeVehicle(vehicleId);
    
    // Remove from vehicles vector
    vehicles.erase(vehicles.begin() + index);
    vehicleIdToIndex.erase(it);
    
    // Update indices
    for (size_t i = index; i < vehicles.size(); i++) {
        vehicleIdToIndex[vehicles[i].vehicleId] = i;
    }
}

Auto* VehicleController::getVehicle(int vehicleId) {
    auto it = vehicleIdToIndex.find(vehicleId);
    return (it != vehicleIdToIndex.end()) ? &vehicles[it->second] : nullptr;
}

const Auto* VehicleController::getVehicle(int vehicleId) const {
    auto it = vehicleIdToIndex.find(vehicleId);
    return (it != vehicleIdToIndex.end()) ? &vehicles[it->second] : nullptr;
}

void VehicleController::setVehicleTarget(int vehicleId, const Point& targetPosition) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;
    
    int targetNodeId = pathSystem->findNearestNode(targetPosition);
    if (targetNodeId != -1) {
        setVehicleTarget(vehicleId, targetNodeId);
    }
}

void VehicleController::setVehicleTarget(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;
    
    vehicle->targetNodeId = targetNodeId;
    planPath(vehicleId, targetNodeId);
}

void VehicleController::updateVehicles(float deltaTime) {
    for (Auto& vehicle : vehicles) {
        updateVehicleMovement(vehicle, deltaTime);
    }
    
    // Update segment manager queues
    segmentManager->updateQueues();
}

bool VehicleController::isVehicleMoving(int vehicleId) const {
    const Auto* vehicle = getVehicle(vehicleId);
    return vehicle && vehicle->state == VehicleState::MOVING;
}

bool VehicleController::hasVehicleArrived(int vehicleId) const {
    const Auto* vehicle = getVehicle(vehicleId);
    return vehicle && vehicle->state == VehicleState::ARRIVED;
}

std::vector<int> VehicleController::getVehiclesAtPosition(const Point& position, float radius) const {
    std::vector<int> nearbyVehicles;
    
    for (const Auto& vehicle : vehicles) {
        if (vehicle.position.distanceTo(position) <= radius) {
            nearbyVehicles.push_back(vehicle.vehicleId);
        }
    }
    
    return nearbyVehicles;
}

bool VehicleController::planPath(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return false;
    
    std::vector<int> path = segmentManager->findAvailablePath(
        vehicle->currentNodeId, targetNodeId, vehicleId);
    
    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->targetNodeId = targetNodeId;
        vehicle->state = VehicleState::MOVING;
        return true;
    }
    
    return false;
}

void VehicleController::clearPath(int vehicleId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;
    
    vehicle->currentPath.clear();
    vehicle->currentSegmentIndex = 0;
    vehicle->state = VehicleState::IDLE;
    
    // Release current segment
    releaseCurrentSegment(*vehicle);
}

bool VehicleController::isPathBlocked(int vehicleId) const {
    const Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentPath.empty()) return false;
    
    // Check if current segment is still available
    if (vehicle->currentSegmentIndex < vehicle->currentPath.size()) {
        int segmentId = vehicle->currentPath[vehicle->currentSegmentIndex];
        return !segmentManager->canVehicleEnterSegment(segmentId, vehicleId);
    }
    
    return false;
}

void VehicleController::updateVehicleMovement(Auto& vehicle, float deltaTime) {
    switch (vehicle.state) {
        case VehicleState::IDLE:
            // Vehicle is not moving
            break;
            
        case VehicleState::MOVING:
            moveVehicleAlongPath(vehicle, deltaTime);
            break;
            
        case VehicleState::WAITING:
            // Try to reserve next segment
            if (tryReserveNextSegment(vehicle)) {
                vehicle.state = VehicleState::MOVING;
            }
            break;
            
        case VehicleState::ARRIVED:
            // Vehicle has reached destination
            break;
    }
}

void VehicleController::moveVehicleAlongPath(Auto& vehicle, float deltaTime) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        vehicle.state = VehicleState::ARRIVED;
        return;
    }
    
    int currentSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];
    
    // Try to reserve current segment if not already reserved
    if (!segmentManager->canVehicleEnterSegment(currentSegmentId, vehicle.vehicleId)) {
        if (!segmentManager->reserveSegment(currentSegmentId, vehicle.vehicleId)) {
            vehicle.state = VehicleState::WAITING;
            return;
        }
    }
    
    const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
    if (!segment) return;
    
    // Get start and end positions
    const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
    const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
    if (!startNode || !endNode) return;
    
    Point segmentStart = startNode->position;
    Point segmentEnd = endNode->position;
    
    // If we're at the wrong end of the segment, swap
    if (vehicle.position.distanceTo(segmentEnd) < vehicle.position.distanceTo(segmentStart)) {
        std::swap(segmentStart, segmentEnd);
    }
    
    // Calculate movement
    float distance = vehicle.speed * deltaTime;
    Point direction = (segmentEnd - segmentStart).normalize();
    Point newPosition = vehicle.position + direction * distance;
    
    // Check if we've reached the end of this segment
    float distanceToEnd = vehicle.position.distanceTo(segmentEnd);
    if (distance >= distanceToEnd) {
        // Snap to end position
        vehicle.position = segmentEnd;
        vehicle.currentNodeId = endNode->nodeId;
        
        // Release current segment
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
        
        // Move to next segment
        vehicle.currentSegmentIndex++;
        
        if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
            // Arrived at destination
            vehicle.state = VehicleState::ARRIVED;
        }
    } else {
        // Continue moving along segment
        vehicle.position = newPosition;
    }
}

Point VehicleController::interpolatePosition(const Point& start, const Point& end, float t) const {
    t = std::max(0.0f, std::min(1.0f, t)); // Clamp t to [0, 1]
    return start + (end - start) * t;
}

bool VehicleController::tryReserveNextSegment(Auto& vehicle) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return false;
    }
    
    int segmentId = vehicle.currentPath[vehicle.currentSegmentIndex];
    return segmentManager->reserveSegment(segmentId, vehicle.vehicleId);
}

void VehicleController::releaseCurrentSegment(Auto& vehicle) {
    int currentSegment = segmentManager->getVehicleSegment(vehicle.vehicleId);
    if (currentSegment != -1) {
        segmentManager->releaseSegment(currentSegment, vehicle.vehicleId);
    }
}