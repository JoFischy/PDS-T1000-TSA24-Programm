
#include "vehicle_controller.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <thread>
#include <chrono>
#include <unordered_map>

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

    // Vehicle starts without target - will stay at spawn until target is set
    vehicle.targetNodeId = -1;
    vehicle.state = VehicleState::ARRIVED;

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

    std::cout << "Spawned " << vehicles.size() << " initial vehicles (no targets assigned)" << std::endl;
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

        setVehicleTargetNode(vehicle.vehicleId, targetNodeId);
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
        setVehicleTargetNode(vehicleId, targetNodeId);
    }
}

void VehicleController::setVehicleTarget(int vehicleId, int targetNodeId) {
    setVehicleTargetNode(vehicleId, targetNodeId);
}

void VehicleController::setVehicleTargetNode(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;

    // First release any occupied segments
    releaseCurrentSegment(*vehicle);

    // If vehicle is not at a node, move it to the nearest node first
    if (vehicle->currentNodeId == -1) {
        int nearestNodeId = pathSystem->findNearestNode(vehicle->position);
        if (nearestNodeId != -1) {
            vehicle->currentNodeId = nearestNodeId;
            const PathNode* node = pathSystem->getNode(nearestNodeId);
            if (node) {
                vehicle->position = node->position;
            }
            std::cout << "Vehicle " << vehicleId << " snapped to nearest node " << nearestNodeId << std::endl;
        }
    }

    // If vehicle is moving and gets new target, first go to current target node
    if (vehicle->state == VehicleState::MOVING && !vehicle->currentPath.empty()) {
        // Find the target node of current segment
        int currentSegmentId = vehicle->currentPath[vehicle->currentSegmentIndex];
        const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
        if (segment) {
            // Determine which end of segment we're going to
            const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
            const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
            
            if (startNode && endNode) {
                float distToStart = vehicle->position.distanceTo(startNode->position);
                float distToEnd = vehicle->position.distanceTo(endNode->position);
                
                // Go to the node we're closer to
                int intermediateTarget = (distToStart < distToEnd) ? segment->startNodeId : segment->endNodeId;
                
                // Clear current path and set intermediate target
                vehicle->currentPath.clear();
                vehicle->currentSegmentIndex = 0;
                vehicle->targetNodeId = intermediateTarget;
                vehicle->pendingTargetNodeId = targetNodeId; // Store final target
                
                // Plan path to intermediate node first
                if (planPath(vehicleId, intermediateTarget)) {
                    vehicle->state = VehicleState::IDLE;
                    std::cout << "Vehicle " << vehicleId << " will first go to node " << intermediateTarget 
                              << " then to final target " << targetNodeId << std::endl;
                } else {
                    vehicle->state = VehicleState::WAITING;
                }
                return;
            }
        }
    }

    vehicle->targetNodeId = targetNodeId;
    vehicle->pendingTargetNodeId = -1; // Clear any pending target
    clearPath(vehicleId);

    // Plan new route if target is different from current position
    if (vehicle->currentNodeId != -1 && vehicle->currentNodeId != targetNodeId) {
        if (planPath(vehicleId, targetNodeId)) {
            vehicle->state = VehicleState::IDLE;
        } else {
            vehicle->state = VehicleState::WAITING;
        }
    } else {
        // Already at target
        vehicle->state = VehicleState::ARRIVED;
    }

    std::cout << "Vehicle " << vehicleId << " new target set to node " << targetNodeId << std::endl;
}

void VehicleController::assignNewRandomTarget(int vehicleId) {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;

    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;

    const auto& nodes = pathSystem->getNodes();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, nodes.size() - 1);

    // Find a different target node than current one
    int targetNodeId;
    do {
        int randomIndex = nodeDist(gen);
        targetNodeId = nodes[randomIndex].nodeId;
    } while (targetNodeId == vehicle->currentNodeId && nodes.size() > 1);

    setVehicleTargetNode(vehicleId, targetNodeId);
}

bool VehicleController::isVehicleAtTarget(int vehicleId) const {
    const Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return false;

    return vehicle->currentNodeId == vehicle->targetNodeId || 
           vehicle->state == VehicleState::ARRIVED;
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

    // Release any currently occupied segments first
    releaseCurrentSegment(*vehicle);

    // Ensure vehicle is at a valid node
    if (vehicle->currentNodeId == -1) {
        int nearestNodeId = pathSystem->findNearestNode(vehicle->position);
        if (nearestNodeId != -1) {
            vehicle->currentNodeId = nearestNodeId;
            const PathNode* node = pathSystem->getNode(nearestNodeId);
            if (node) {
                vehicle->position = node->position;
            }
            std::cout << "Vehicle " << vehicleId << " moved to nearest node " << nearestNodeId << std::endl;
        } else {
            return false;
        }
    }

    // Already at target?
    if (vehicle->currentNodeId == targetNodeId) {
        vehicle->state = VehicleState::ARRIVED;
        vehicle->currentPath.clear();
        return true;
    }

    // Find path
    std::vector<int> path = pathSystem->findPath(vehicle->currentNodeId, targetNodeId, {});

    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->targetNodeId = targetNodeId;
        vehicle->state = VehicleState::IDLE;
        std::cout << "Vehicle " << vehicleId << " planned path with " << path.size() << " segments" << std::endl;
        return true;
    } else {
        vehicle->state = VehicleState::WAITING;
        std::cout << "Vehicle " << vehicleId << " no path found to target" << std::endl;
        return false;
    }
}

bool VehicleController::replanPathIfBlocked(int vehicleId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->targetNodeId == -1) return false;

    // Clear current route and plan new one
    clearPath(vehicleId);
    return planPath(vehicleId, vehicle->targetNodeId);
}

bool VehicleController::findAlternativePath(int vehicleId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1 || vehicle->targetNodeId == -1) return false;

    // Try to find available path
    std::vector<int> path = segmentManager->findAvailablePath(
        vehicle->currentNodeId, vehicle->targetNodeId, vehicleId);

    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->state = VehicleState::IDLE;
        return true;
    }

    return false;
}

void VehicleController::handleBlockedVehicle(Auto& vehicle) {
    if (vehicle.state != VehicleState::WAITING) return;

    static std::unordered_map<int, float> waitTimers;
    static std::unordered_map<int, int> retryCount;

    waitTimers[vehicle.vehicleId] += 0.016f; // ca. 60 FPS

    // Wait 1 second before retry
    if (waitTimers[vehicle.vehicleId] > 1.0f) {
        waitTimers[vehicle.vehicleId] = 0.0f;
        retryCount[vehicle.vehicleId]++;

        // Try to reserve current segment again
        if (!vehicle.currentPath.empty() && 
            vehicle.currentSegmentIndex < vehicle.currentPath.size()) {

            int currentSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];
            if (segmentManager->reserveSegment(currentSegmentId, vehicle.vehicleId)) {
                vehicle.state = VehicleState::MOVING;
                retryCount[vehicle.vehicleId] = 0;
                std::cout << "Vehicle " << vehicle.vehicleId << " successfully reserved segment " << currentSegmentId << std::endl;
                return;
            }
        }

        // After 3 attempts, try to replan
        if (retryCount[vehicle.vehicleId] > 3) {
            retryCount[vehicle.vehicleId] = 0;
            std::cout << "Vehicle " << vehicle.vehicleId << " replanning path after timeout" << std::endl;
            replanPathIfBlocked(vehicle.vehicleId);
        }
    }
}

void VehicleController::clearPath(int vehicleId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;

    vehicle->currentPath.clear();
    vehicle->currentSegmentIndex = 0;

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
    static int debugCounter = 0;
    bool debug = (debugCounter++ % 180 == 0); // Debug every 3 seconds at 60 FPS

    switch (vehicle.state) {
        case VehicleState::IDLE:
            if (debug) {
                std::cout << "Vehicle " << vehicle.vehicleId << " IDLE at node " 
                         << vehicle.currentNodeId << " target " << vehicle.targetNodeId << std::endl;
            }
            // Vehicle waits for new task or next segment
            if (vehicle.targetNodeId != -1 && vehicle.currentNodeId != vehicle.targetNodeId) {
                if (!vehicle.currentPath.empty() && vehicle.currentSegmentIndex < vehicle.currentPath.size()) {
                    // Has path - try to continue movement
                    moveVehicleAlongPath(vehicle, deltaTime);
                } else {
                    // Plan new path
                    if (debug) {
                        std::cout << "Vehicle " << vehicle.vehicleId << " planning new path" << std::endl;
                    }
                    if (!planPath(vehicle.vehicleId, vehicle.targetNodeId)) {
                        vehicle.state = VehicleState::WAITING;
                    }
                }
            } else if (vehicle.targetNodeId == -1) {
                // No target set - stay at current position
                vehicle.state = VehicleState::ARRIVED;
            }
            break;

        case VehicleState::MOVING:
            moveVehicleAlongPath(vehicle, deltaTime);
            break;

        case VehicleState::WAITING:
            // Handle blocked vehicles
            handleBlockedVehicle(vehicle);
            break;

        case VehicleState::ARRIVED:
            // Vehicle stays at destination until new target is set
            if (debug && vehicle.targetNodeId != -1) {
                std::cout << "Vehicle " << vehicle.vehicleId << " arrived at target " << vehicle.targetNodeId << std::endl;
            }
            // Don't assign new targets automatically - wait for manual assignment
            break;
    }
}

void VehicleController::moveVehicleAlongPath(Auto& vehicle, float deltaTime) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        
        // Check if we have a pending target (after reaching intermediate node)
        if (vehicle.pendingTargetNodeId != -1) {
            int pendingTarget = vehicle.pendingTargetNodeId;
            vehicle.pendingTargetNodeId = -1;
            vehicle.targetNodeId = pendingTarget;
            
            if (planPath(vehicle.vehicleId, pendingTarget)) {
                vehicle.state = VehicleState::IDLE;
                std::cout << "Vehicle " << vehicle.vehicleId << " now continuing to final target " << pendingTarget << std::endl;
                return;
            }
        }
        
        vehicle.state = VehicleState::ARRIVED;
        return;
    }

    int currentSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];

    // Get segment information
    const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
    if (!segment) {
        vehicle.state = VehicleState::WAITING;
        return;
    }

    // Check for collision risks ahead before moving
    if (checkCollisionRisk(vehicle, currentSegmentId)) {
        vehicle.state = VehicleState::WAITING;
        vehicle.isWaitingAtSafetyStop = true;
        std::cout << "Vehicle " << vehicle.vehicleId << " stopping at safety point - collision risk detected" << std::endl;
        return;
    }

    // Try to reserve segment if not already reserved
    if (!segmentManager->canVehicleEnterSegment(currentSegmentId, vehicle.vehicleId)) {
        if (!segmentManager->reserveSegment(currentSegmentId, vehicle.vehicleId)) {
            vehicle.state = VehicleState::WAITING;
            vehicle.isWaitingAtSafetyStop = false;
            std::cout << "Vehicle " << vehicle.vehicleId << " waiting for segment " << currentSegmentId << std::endl;
            return;
        }
    }

    // We can move on this segment
    vehicle.state = VehicleState::MOVING;
    vehicle.isWaitingAtSafetyStop = false;

    const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
    const PathNode* endNode = pathSystem->getNode(segment->endNodeId);

    if (!startNode || !endNode) {
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
        vehicle.state = VehicleState::WAITING;
        return;
    }

    // Determine target position based on path direction
    Point targetPos;
    int targetNodeId;

    // Look ahead to next segment to determine direction
    if (vehicle.currentSegmentIndex + 1 < vehicle.currentPath.size()) {
        int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex + 1];
        const PathSegment* nextSegment = pathSystem->getSegment(nextSegmentId);

        if (nextSegment) {
            // Find shared node between current and next segment
            int sharedNode = -1;
            if (segment->startNodeId == nextSegment->startNodeId || segment->startNodeId == nextSegment->endNodeId) {
                sharedNode = segment->startNodeId;
            } else if (segment->endNodeId == nextSegment->startNodeId || segment->endNodeId == nextSegment->endNodeId) {
                sharedNode = segment->endNodeId;
            }

            if (sharedNode != -1) {
                // Drive to shared node
                const PathNode* sharedNodePtr = pathSystem->getNode(sharedNode);
                if (sharedNodePtr) {
                    targetPos = sharedNodePtr->position;
                    targetNodeId = sharedNode;
                }
            } else {
                // Fallback: drive to nearest end
                float distToStart = vehicle.position.distanceTo(startNode->position);
                float distToEnd = vehicle.position.distanceTo(endNode->position);
                if (distToStart > distToEnd) {
                    targetPos = endNode->position;
                    targetNodeId = endNode->nodeId;
                } else {
                    targetPos = startNode->position;
                    targetNodeId = startNode->nodeId;
                }
            }
        }
    } else {
        // Last segment - drive to target node
        if (vehicle.targetNodeId == startNode->nodeId) {
            targetPos = startNode->position;
            targetNodeId = startNode->nodeId;
        } else if (vehicle.targetNodeId == endNode->nodeId) {
            targetPos = endNode->position;
            targetNodeId = endNode->nodeId;
        } else {
            // Drive to farther end
            float distToStart = vehicle.position.distanceTo(startNode->position);
            float distToEnd = vehicle.position.distanceTo(endNode->position);
            if (distToStart > distToEnd) {
                targetPos = endNode->position;
                targetNodeId = endNode->nodeId;
            } else {
                targetPos = startNode->position;
                targetNodeId = startNode->nodeId;
            }
        }
    }

    vehicle.targetPosition = targetPos;

    // Calculate movement
    float moveDistance = vehicle.speed * deltaTime;
    Point direction = (targetPos - vehicle.position).normalize();
    float distanceToTarget = vehicle.position.distanceTo(targetPos);

    if (moveDistance >= distanceToTarget || distanceToTarget < 5.0f) {
        // Arrived at segment target node
        vehicle.position = targetPos;
        vehicle.currentNodeId = targetNodeId;

        // Release segment immediately when target node is reached
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);

        // Move to next segment
        vehicle.currentSegmentIndex++;

        if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
            // Reached final target
            vehicle.state = VehicleState::ARRIVED;
            vehicle.currentPath.clear();
            std::cout << "Vehicle " << vehicle.vehicleId << " arrived at final target node " << vehicle.targetNodeId << std::endl;
        } else {
            // Back to IDLE for next segment
            vehicle.state = VehicleState::IDLE;
            std::cout << "Vehicle " << vehicle.vehicleId << " reached node " << targetNodeId 
                      << ", preparing for next segment " << vehicle.currentSegmentIndex << std::endl;
        }
    } else {
        // Continue moving towards target node
        Point newPosition = vehicle.position + direction * moveDistance;
        vehicle.position = newPosition;
    }
}

bool VehicleController::checkCollisionRisk(const Auto& vehicle, int segmentId) {
    // Check if next 2 segments in path are occupied by other vehicles
    if (vehicle.currentSegmentIndex + 1 >= vehicle.currentPath.size()) {
        return false; // Last segment, no collision risk ahead
    }

    int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex + 1];
    const PathSegment* nextSegment = pathSystem->getSegment(nextSegmentId);
    
    if (nextSegment && nextSegment->isOccupied && 
        nextSegment->occupiedByVehicleId != vehicle.vehicleId) {
        return true; // Next segment occupied
    }

    // Check segment after next as well for better collision avoidance
    if (vehicle.currentSegmentIndex + 2 < vehicle.currentPath.size()) {
        int nextNextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex + 2];
        const PathSegment* nextNextSegment = pathSystem->getSegment(nextNextSegmentId);
        
        if (nextNextSegment && nextNextSegment->isOccupied && 
            nextNextSegment->occupiedByVehicleId != vehicle.vehicleId) {
            return true; // Segment after next occupied
        }
    }

    return false;
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
