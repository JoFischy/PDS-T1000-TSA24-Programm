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
    
    // Filter out waiting nodes - only use regular nodes as targets
    std::vector<int> validTargetNodes;
    for (const auto& node : nodes) {
        if (!node.isWaitingNode) {
            validTargetNodes.push_back(node.nodeId);
        }
    }
    
    if (validTargetNodes.empty()) {
        std::cout << "No valid target nodes available (no non-waiting nodes)" << std::endl;
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, validTargetNodes.size() - 1);

    for (auto& vehicle : vehicles) {
        // Find a different target node than current one
        int targetNodeId;
        do {
            int randomIndex = nodeDist(gen);
            targetNodeId = validTargetNodes[randomIndex];
        } while (targetNodeId == vehicle.currentNodeId && validTargetNodes.size() > 1);

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

    // Check if target node is a waiting node - not allowed as target
    const PathNode* targetNode = pathSystem->getNode(targetNodeId);
    if (targetNode && targetNode->isWaitingNode) {
        std::cout << "Vehicle " << vehicleId << " cannot target waiting node " << targetNodeId << std::endl;
        return;
    }

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
    
    // Filter out waiting nodes - only use regular nodes as targets
    std::vector<int> validTargetNodes;
    for (const auto& node : nodes) {
        if (!node.isWaitingNode) {
            validTargetNodes.push_back(node.nodeId);
        }
    }
    
    if (validTargetNodes.empty()) {
        std::cout << "No valid target nodes available for vehicle " << vehicleId << std::endl;
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, validTargetNodes.size() - 1);

    // Find a different target node than current one
    int targetNodeId;
    do {
        int randomIndex = nodeDist(gen);
        targetNodeId = validTargetNodes[randomIndex];
    } while (targetNodeId == vehicle->currentNodeId && validTargetNodes.size() > 1);

    setVehicleTargetNode(vehicleId, targetNodeId);
}

bool VehicleController::isVehicleAtTarget(int vehicleId) const {
    const Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return false;

    return vehicle->currentNodeId == vehicle->targetNodeId ||
           vehicle->state == VehicleState::ARRIVED;
}

void VehicleController::updateVehicles(float deltaTime) {
    // First coordinate movements between vehicles
    coordinateVehicleMovements();

    for (Auto& vehicle : vehicles) {
        updateVehicleMovement(vehicle, deltaTime);
    }

    // Update segment manager queues
    segmentManager->updateQueues();
}

void VehicleController::coordinateVehicleMovements() {
    // Detect upcoming conflicts between all vehicles
    auto upcomingConflicts = detectUpcomingConflicts();
    
    // Prevent deadlocks through proactive coordination
    preventDeadlockThroughCoordination();
    
    // Resolve conflicts through negotiation
    for (const auto& conflict : upcomingConflicts) {
        if (conflict.canNegotiate) {
            resolveConflictThroughNegotiation(conflict);
        }
    }
    
    // Process vehicles that are waiting and try to help them find alternative routes
    for (Auto& vehicle : vehicles) {
        if (vehicle.state == VehicleState::WAITING && vehicle.targetNodeId != -1) {
            // Check if vehicle should continue waiting due to conflicts
            if (shouldVehicleWaitForConflictResolution(vehicle.vehicleId, upcomingConflicts)) {
                continue; // Keep waiting
            }
            
            // Check if current path is still valid or if we need to recalculate
            if (!vehicle.currentPath.empty()) {
                int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];

                // If segment is occupied by another vehicle, try to find alternative
                const PathSegment* segment = pathSystem->getSegment(nextSegmentId);
                if (segment && segment->isOccupied &&
                    segment->occupiedByVehicleId != vehicle.vehicleId) {

                    // Try to find alternative path
                    std::vector<int> altPath = segmentManager->findAvailablePath(
                        vehicle.currentNodeId, vehicle.targetNodeId, vehicle.vehicleId);

                    if (!altPath.empty() && altPath != vehicle.currentPath) {
                        std::cout << "Vehicle " << vehicle.vehicleId
                                  << " switching to alternative path" << std::endl;
                        clearPath(vehicle.vehicleId);
                        vehicle.currentPath = altPath;
                        vehicle.currentSegmentIndex = 0;
                        vehicle.state = VehicleState::IDLE;
                    }
                }
            }
        }
    }
}

std::vector<VehicleController::VehicleConflict> VehicleController::detectUpcomingConflicts() const {
    std::vector<VehicleConflict> conflicts;
    
    // Check all pairs of vehicles for potential conflicts
    for (size_t i = 0; i < vehicles.size(); i++) {
        for (size_t j = i + 1; j < vehicles.size(); j++) {
            const Auto& vehicle1 = vehicles[i];
            const Auto& vehicle2 = vehicles[j];
            
            if (vehicle1.currentPath.empty() || vehicle2.currentPath.empty()) continue;
            
            // Find common junctions in their paths
            int commonJunction = findCommonJunctionInPaths(vehicle1, vehicle2);
            if (commonJunction != -1) {
                VehicleConflict conflict;
                conflict.vehicleId1 = vehicle1.vehicleId;
                conflict.vehicleId2 = vehicle2.vehicleId;
                conflict.conflictJunctionId = commonJunction;
                conflict.estimatedConflictTime = estimateTimeToJunction(vehicle1, commonJunction);
                conflict.canNegotiate = (conflict.estimatedConflictTime > 2.0f); // Can negotiate if >2 seconds away
                
                conflicts.push_back(conflict);
            }
        }
    }
    
    return conflicts;
}

int VehicleController::findCommonJunctionInPaths(const Auto& vehicle1, const Auto& vehicle2) const {
    // Check each segment in vehicle1's path
    for (size_t i = vehicle1.currentSegmentIndex; i < vehicle1.currentPath.size(); i++) {
        int segmentId1 = vehicle1.currentPath[i];
        const PathSegment* segment1 = pathSystem->getSegment(segmentId1);
        if (!segment1) continue;
        
        // Check if either endpoint is a junction
        for (int nodeId : {segment1->startNodeId, segment1->endNodeId}) {
            if (segmentManager->isJunctionNode(nodeId)) {
                // Check if vehicle2's path also includes this junction
                for (size_t j = vehicle2.currentSegmentIndex; j < vehicle2.currentPath.size(); j++) {
                    int segmentId2 = vehicle2.currentPath[j];
                    const PathSegment* segment2 = pathSystem->getSegment(segmentId2);
                    if (!segment2) continue;
                    
                    if (segment2->startNodeId == nodeId || segment2->endNodeId == nodeId) {
                        return nodeId; // Found common junction
                    }
                }
            }
        }
    }
    
    return -1; // No common junction found
}

float VehicleController::estimateTimeToJunction(const Auto& vehicle, int junctionId) const {
    float totalTime = 0.0f;
    
    for (size_t i = vehicle.currentSegmentIndex; i < vehicle.currentPath.size(); i++) {
        int segmentId = vehicle.currentPath[i];
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        totalTime += segment->length / vehicle.speed;
        
        // Check if this segment leads to the target junction
        if (segment->startNodeId == junctionId || segment->endNodeId == junctionId) {
            return totalTime;
        }
    }
    
    return std::numeric_limits<float>::max(); // Junction not found in path
}

bool VehicleController::shouldVehicleWaitForConflictResolution(int vehicleId, const std::vector<VehicleConflict>& conflicts) const {
    for (const auto& conflict : conflicts) {
        if ((conflict.vehicleId1 == vehicleId || conflict.vehicleId2 == vehicleId) && 
            conflict.estimatedConflictTime < 3.0f) {
            return true; // Wait if conflict is imminent
        }
    }
    return false;
}

bool VehicleController::resolveConflictThroughNegotiation(const VehicleConflict& conflict) {
    Auto* vehicle1 = getVehicle(conflict.vehicleId1);
    Auto* vehicle2 = getVehicle(conflict.vehicleId2);
    
    if (!vehicle1 || !vehicle2) return false;
    
    // Simple negotiation: vehicle with higher ID yields to lower ID
    if (conflict.vehicleId1 < conflict.vehicleId2) {
        // Vehicle2 should find alternative route or wait
        if (vehicle2->state != VehicleState::WAITING) {
            std::cout << "Vehicle " << conflict.vehicleId2 << " yielding to vehicle " << conflict.vehicleId1 << " at junction " << conflict.conflictJunctionId << std::endl;
            
            // Try to find alternative route
            if (!findAlternativePath(conflict.vehicleId2)) {
                // If no alternative, move to nearest waiting node
                const PathNode* currentNode = pathSystem->getNode(vehicle2->currentNodeId);
                if (!currentNode || !currentNode->isWaitingNode) {
                    moveToNearestWaitingNode(*vehicle2);
                }
            }
        }
    } else {
        // Vehicle1 should find alternative route or wait
        if (vehicle1->state != VehicleState::WAITING) {
            std::cout << "Vehicle " << conflict.vehicleId1 << " yielding to vehicle " << conflict.vehicleId2 << " at junction " << conflict.conflictJunctionId << std::endl;
            
            // Try to find alternative route
            if (!findAlternativePath(conflict.vehicleId1)) {
                // If no alternative, move to nearest waiting node
                const PathNode* currentNode = pathSystem->getNode(vehicle1->currentNodeId);
                if (!currentNode || !currentNode->isWaitingNode) {
                    moveToNearestWaitingNode(*vehicle1);
                }
            }
        }
    }
    
    return true;
}

void VehicleController::preventDeadlockThroughCoordination() {
    // Detect potential circular dependencies
    std::vector<int> waitingVehicles;
    
    for (const Auto& vehicle : vehicles) {
        if (vehicle.state == VehicleState::WAITING) {
            waitingVehicles.push_back(vehicle.vehicleId);
        }
    }
    
    // If too many vehicles are waiting, force some to find alternatives
    if (waitingVehicles.size() > vehicles.size() / 2) {
        std::cout << "Potential deadlock detected, forcing alternative routes" << std::endl;
        
        for (size_t i = 0; i < waitingVehicles.size() / 2; i++) {
            int vehicleId = waitingVehicles[i];
            Auto* vehicle = getVehicle(vehicleId);
            if (vehicle) {
                assignNewRandomTarget(vehicleId);
            }
        }
    }
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

    // Check if target node is a waiting node - not allowed as target
    const PathNode* targetNode = pathSystem->getNode(targetNodeId);
    if (targetNode && targetNode->isWaitingNode) {
        std::cout << "Vehicle " << vehicleId << " cannot plan path to waiting node " << targetNodeId << std::endl;
        return false;
    }

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

    // First try to find path avoiding currently occupied segments
    std::vector<int> path = segmentManager->findAvailablePath(vehicle->currentNodeId, targetNodeId, vehicleId);

    // If no available path found, use optimal path (vehicles will wait for segments)
    if (path.empty()) {
        path = segmentManager->findOptimalPath(vehicle->currentNodeId, targetNodeId, vehicleId);
    }

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

    // Try to find available path using waiting points
    std::vector<int> path = segmentManager->findAvailablePath(
        vehicle->currentNodeId, vehicle->targetNodeId, vehicleId);

    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->state = VehicleState::IDLE;
        std::cout << "Vehicle " << vehicleId << " found alternative path with waiting points" << std::endl;
        return true;
    }

    return false;
}

void VehicleController::handleBlockedVehicle(Auto& vehicle) {
    if (vehicle.state != VehicleState::WAITING) return;

    static std::unordered_map<int, int> retryCount;

    // Try to reserve next segment
    if (!vehicle.currentPath.empty() && vehicle.currentSegmentIndex < vehicle.currentPath.size()) {
        int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];

        if (segmentManager->canVehicleEnterSegment(nextSegmentId, vehicle.vehicleId)) {
            if (segmentManager->reserveSegment(nextSegmentId, vehicle.vehicleId)) {
                vehicle.state = VehicleState::MOVING;
                retryCount[vehicle.vehicleId] = 0;
                std::cout << "Vehicle " << vehicle.vehicleId << " successfully reserved segment " << nextSegmentId << std::endl;
                return;
            }
        }

        // Add to queue if not already there
        segmentManager->addToQueue(nextSegmentId, vehicle.vehicleId);
    }

    retryCount[vehicle.vehicleId]++;

    // After 5 attempts, try to find alternative path
    if (retryCount[vehicle.vehicleId] > 5) {
        retryCount[vehicle.vehicleId] = 0;
        std::cout << "Vehicle " << vehicle.vehicleId << " searching for alternative path" << std::endl;

        // Try to find alternative path that avoids currently occupied segments
        if (findAlternativePath(vehicle.vehicleId)) {
            vehicle.state = VehicleState::IDLE;
            std::cout << "Vehicle " << vehicle.vehicleId << " found alternative path" << std::endl;
        } else {
            // If no alternative found, replan with optimal path
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

// Removed old safety stop logic - using new waiting point system

void VehicleController::updateVehicleMovement(Auto& vehicle, float deltaTime) {
    static int debugCounter = 0;
    static std::unordered_map<int, float> rerouteTimers;
    bool debug = (debugCounter++ % 180 == 0); // Debug every 3 seconds at 60 FPS

    // Increment reroute timer
    rerouteTimers[vehicle.vehicleId] += deltaTime;

    switch (vehicle.state) {
        case VehicleState::IDLE:
            if (debug) {
                std::cout << "Vehicle " << vehicle.vehicleId << " IDLE at node "
                         << vehicle.currentNodeId << " target " << vehicle.targetNodeId << std::endl;
            }

            // Vehicle waits for new task or next segment
            if (vehicle.targetNodeId != -1 && vehicle.currentNodeId != vehicle.targetNodeId) {
                // Check for upcoming conflicts and minimum distance
                const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
                
                // Check minimum distance (reduced to 100 pixels)
                if (currentNode && currentNode->isWaitingNode && !hasMinimumDistanceToOtherVehicles(vehicle, 100.0f)) {
                    vehicle.state = VehicleState::WAITING;
                    if (debug) {
                        std::cout << "Vehicle " << vehicle.vehicleId << " waiting for minimum distance at waiting node" << std::endl;
                    }
                    break;
                }
                
                // Advanced conflict detection
                if (currentNode && currentNode->isWaitingNode) {
                    auto conflicts = segmentManager->detectPotentialConflicts(vehicle.vehicleId, vehicle.currentPath);
                    if (segmentManager->shouldWaitAtWaitingNode(vehicle.vehicleId, conflicts)) {
                        vehicle.state = VehicleState::WAITING;
                        if (debug) {
                            std::cout << "Vehicle " << vehicle.vehicleId << " waiting due to predicted conflicts" << std::endl;
                        }
                        break;
                    }
                }

                // Recalculate route every 2 seconds or if no path exists
                if (rerouteTimers[vehicle.vehicleId] > 2.0f || vehicle.currentPath.empty()) {
                    rerouteTimers[vehicle.vehicleId] = 0.0f;
                    if (debug) {
                        std::cout << "Vehicle " << vehicle.vehicleId << " recalculating path" << std::endl;
                    }
                    clearPath(vehicle.vehicleId);
                    if (!planPath(vehicle.vehicleId, vehicle.targetNodeId)) {
                        vehicle.state = VehicleState::WAITING;
                        break;
                    }
                }

                if (!vehicle.currentPath.empty() && vehicle.currentSegmentIndex < vehicle.currentPath.size()) {
                    // Check if vehicle must wait at waiting node before proceeding to junction
                    if (mustWaitAtWaitingNodeForBlockedPath(vehicle)) {
                        vehicle.state = VehicleState::WAITING;
                        if (debug) {
                            std::cout << "Vehicle " << vehicle.vehicleId << " must wait at waiting node - path ahead blocked" << std::endl;
                        }
                        break;
                    }

                    // Try to reserve next segment
                    int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];
                    if (segmentManager->canVehicleEnterSegment(nextSegmentId, vehicle.vehicleId)) {
                        if (segmentManager->reserveSegment(nextSegmentId, vehicle.vehicleId)) {
                            vehicle.state = VehicleState::MOVING;
                        } else {
                            // Can only wait at waiting nodes
                            if (currentNode && currentNode->isWaitingNode) {
                                vehicle.state = VehicleState::WAITING;
                                segmentManager->addToQueue(nextSegmentId, vehicle.vehicleId);
                            } else {
                                // Force movement to nearest waiting node
                                moveToNearestWaitingNode(vehicle);
                            }
                        }
                    } else {
                        // Can only wait at waiting nodes
                        if (currentNode && currentNode->isWaitingNode) {
                            vehicle.state = VehicleState::WAITING;
                            segmentManager->addToQueue(nextSegmentId, vehicle.vehicleId);
                        } else {
                            // Force movement to nearest waiting node
                            moveToNearestWaitingNode(vehicle);
                        }
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

        case VehicleState::WAITING: {
            // Vehicles can only wait at waiting nodes
            const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
            if (!currentNode || !currentNode->isWaitingNode) {
                std::cout << "Vehicle " << vehicle.vehicleId << " cannot wait at non-waiting node " << vehicle.currentNodeId << std::endl;
                moveToNearestWaitingNode(vehicle);
                break;
            }

            // Check for deadlock and resolve it
            if (segmentManager->isVehicleDeadlocked(vehicle.vehicleId) || 
                rerouteTimers[vehicle.vehicleId] > 10.0f) { // Force reroute after 10 seconds of waiting
                
                std::cout << "Vehicle " << vehicle.vehicleId << " resolving deadlock/long wait" << std::endl;
                segmentManager->clearDeadlockFlag(vehicle.vehicleId);
                rerouteTimers[vehicle.vehicleId] = 0.0f;
                
                // Try alternative route or assign new target
                if (!findAlternativeRouteFromWaitingNode(vehicle)) {
                    assignNewRandomTarget(vehicle.vehicleId);
                }
                break;
            }
            
            // Handle blocked vehicles - try every second
            if (rerouteTimers[vehicle.vehicleId] > 1.0f) {
                rerouteTimers[vehicle.vehicleId] = 0.0f;
                handleBlockedVehicle(vehicle);
            }
            break;
        }

        case VehicleState::ARRIVED:
            // Vehicle stays at destination until new target is set
            if (debug && vehicle.targetNodeId != -1) {
                std::cout << "Vehicle " << vehicle.vehicleId << " arrived at target " << vehicle.targetNodeId << std::endl;
            }
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

    // Verify that vehicle has reserved this segment
    if (segment->occupiedByVehicleId != vehicle.vehicleId) {
        vehicle.state = VehicleState::WAITING;
        std::cout << "Vehicle " << vehicle.vehicleId << " cannot move - segment " << currentSegmentId
                  << " not reserved by this vehicle" << std::endl;
        return;
    }

    // We can move on this segment
    vehicle.state = VehicleState::MOVING;

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
    bool isReverseMovement = false;

    // Look ahead to next segment to determine direction
    if (vehicle.currentSegmentIndex + 1 < vehicle.currentPath.size()) {
        int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex + 1];
        const PathSegment* nextSegment = pathSystem->getSegment(nextSegmentId);

        if (nextSegment) {
            // Normal forward movement - find shared node between current and next segment
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

    // Check if movement would violate minimum distance
    Point newPosition = vehicle.position + direction * std::min(moveDistance, distanceToTarget);
    if (!canMoveWithoutViolatingDistance(vehicle, newPosition, 100.0f)) {
        // Cannot move without violating minimum distance - stay at current position
        vehicle.state = VehicleState::WAITING;
        return;
    }

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
        // Continue moving towards target node (newPosition already calculated and validated)
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

bool VehicleController::mustWaitAtWaitingNodeForBlockedPath(const Auto& vehicle) {
    if (vehicle.currentPath.empty() || vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return false;
    }

    // Check if current node is a waiting node
    const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
    if (!currentNode || !currentNode->isWaitingNode) {
        return false; // Not at waiting node, cannot wait here
    }

    // Check if path ahead through next junction is blocked
    return isPathAheadBlocked(vehicle);
}

bool VehicleController::isPathAheadBlocked(const Auto& vehicle) {
    if (vehicle.currentPath.empty() || vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return false;
    }

    // Look ahead in path to find next junction node
    int nextJunctionId = findNextJunctionInPath(vehicle);
    if (nextJunctionId == -1) {
        return false; // No junction found in path
    }

    // Check if segment after the junction is blocked
    int segmentAfterJunction = findSegmentAfterJunction(vehicle, nextJunctionId);
    if (segmentAfterJunction == -1) {
        return false; // No segment after junction found
    }

    const PathSegment* blockedSegment = pathSystem->getSegment(segmentAfterJunction);
    if (blockedSegment && blockedSegment->isOccupied && 
        blockedSegment->occupiedByVehicleId != vehicle.vehicleId) {
        std::cout << "Vehicle " << vehicle.vehicleId << " waiting - segment " << segmentAfterJunction 
                  << " after junction " << nextJunctionId << " is blocked by vehicle " 
                  << blockedSegment->occupiedByVehicleId << std::endl;
        return true;
    }

    // Also check if there are conflicting vehicles at the junction
    return hasConflictAtJunction(nextJunctionId, vehicle.vehicleId);
}

int VehicleController::findNextJunctionInPath(const Auto& vehicle) {
    if (vehicle.currentPath.empty() || vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return -1;
    }

    // Look through remaining path segments to find next junction
    for (size_t i = vehicle.currentSegmentIndex; i < vehicle.currentPath.size(); i++) {
        int segmentId = vehicle.currentPath[i];
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;

        // Check both endpoints of segment
        const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
        const PathNode* endNode = pathSystem->getNode(segment->endNodeId);

        if (startNode && !startNode->isWaitingNode && startNode->connectedSegments.size() >= 3) {
            return startNode->nodeId; // Found junction
        }
        if (endNode && !endNode->isWaitingNode && endNode->connectedSegments.size() >= 3) {
            return endNode->nodeId; // Found junction
        }
    }

    return -1; // No junction found
}

int VehicleController::findSegmentAfterJunction(const Auto& vehicle, int junctionId) {
    if (vehicle.currentPath.empty() || vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return -1;
    }

    // Find the segment that comes after the junction in the path
    bool foundJunction = false;
    for (size_t i = vehicle.currentSegmentIndex; i < vehicle.currentPath.size(); i++) {
        int segmentId = vehicle.currentPath[i];
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;

        if (foundJunction) {
            // This is the segment after the junction
            return segmentId;
        }

        // Check if this segment connects to the junction
        if (segment->startNodeId == junctionId || segment->endNodeId == junctionId) {
            foundJunction = true;
        }
    }

    return -1; // No segment after junction found
}

bool VehicleController::hasConflictAtJunction(int junctionId, int vehicleId) {
    const PathNode* junction = pathSystem->getNode(junctionId);
    if (!junction) return false;

    // Check all segments connected to junction for conflicting vehicles
    int conflictingVehicles = 0;
    for (int segmentId : junction->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;

        if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
            conflictingVehicles++;
        }

        // Also check queued vehicles
        for (int queuedVehicleId : segment->queuedVehicles) {
            if (queuedVehicleId != vehicleId) {
                conflictingVehicles++;
                break;
            }
        }
    }

    return conflictingVehicles > 0;
}

void VehicleController::moveToNearestWaitingNode(Auto& vehicle) {
    // Find nearest waiting node and create path to it
    int nearestWaitingNode = findNearestWaitingNode(vehicle.currentNodeId);
    if (nearestWaitingNode != -1) {
        std::cout << "Vehicle " << vehicle.vehicleId << " forced to move to nearest waiting node " << nearestWaitingNode << std::endl;
        clearPath(vehicle.vehicleId);
        if (planPath(vehicle.vehicleId, nearestWaitingNode)) {
            // Temporarily set waiting node as target
            int originalTarget = vehicle.targetNodeId;
            vehicle.targetNodeId = nearestWaitingNode;
            vehicle.pendingTargetNodeId = originalTarget; // Store original target
            vehicle.state = VehicleState::IDLE;
        }
    }
}

int VehicleController::findNearestWaitingNode(int currentNodeId) {
    const PathNode* currentNode = pathSystem->getNode(currentNodeId);
    if (!currentNode) return -1;

    int nearestWaitingNode = -1;
    float nearestDistance = std::numeric_limits<float>::max();

    // Check all nodes to find nearest waiting node
    for (const auto& node : pathSystem->getNodes()) {
        if (node.isWaitingNode && node.nodeId != currentNodeId) {
            float distance = currentNode->position.distanceTo(node.position);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestWaitingNode = node.nodeId;
            }
        }
    }

    return nearestWaitingNode;
}

bool VehicleController::findAlternativeRouteFromWaitingNode(Auto& vehicle) {
    // Try to find alternative path from current waiting node
    clearPath(vehicle.vehicleId);
    
    // First try to find completely different route
    std::vector<int> altPath = segmentManager->findAvailablePath(
        vehicle.currentNodeId, vehicle.targetNodeId, vehicle.vehicleId);
    
    if (!altPath.empty()) {
        vehicle.currentPath = altPath;
        vehicle.currentSegmentIndex = 0;
        vehicle.state = VehicleState::IDLE;
        std::cout << "Vehicle " << vehicle.vehicleId << " found alternative route from waiting node" << std::endl;
        return true;
    }
    
    return false;
}

int VehicleController::findTJunctionForWaitingNode(int waitingNodeId) {
    // Find the T-junction connected to this waiting node
    const PathNode* waitingNode = pathSystem->getNode(waitingNodeId);
    if (!waitingNode) return -1;

    // Check all connected segments
    for (int segmentId : waitingNode->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;

        int otherNodeId = (segment->startNodeId == waitingNodeId) ? 
                         segment->endNodeId : segment->startNodeId;
        
        // Check if other node is a T-junction
        if (segmentManager->isTJunction(otherNodeId)) {
            return otherNodeId;
        }
    }

    return -1;
}

bool VehicleController::hasConflictAtTJunction(int tJunctionId, int vehicleId) {
    const PathNode* tJunction = pathSystem->getNode(tJunctionId);
    if (!tJunction) return false;

    // Count vehicles approaching or at this T-junction
    int approachingVehicles = 0;
    std::vector<int> conflictingVehicles;

    // Check all segments connected to T-junction
    for (int segmentId : tJunction->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;

        // Check if segment is occupied by another vehicle
        if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
            approachingVehicles++;
            conflictingVehicles.push_back(segment->occupiedByVehicleId);
        }

        // Check if vehicles are queued for this segment
        if (!segment->queuedVehicles.empty()) {
            for (int queuedVehicleId : segment->queuedVehicles) {
                if (queuedVehicleId != vehicleId) {
                    approachingVehicles++;
                    conflictingVehicles.push_back(queuedVehicleId);
                }
            }
        }
    }

    // If there are other vehicles, apply T-junction conflict rules
    if (approachingVehicles > 0) {
        return resolveTJunctionConflict(tJunctionId, vehicleId, conflictingVehicles);
    }

    return false; // No conflict, can proceed
}

bool VehicleController::resolveTJunctionConflict(int tJunctionId, int vehicleId, const std::vector<int>& conflictingVehicles) {
    if (conflictingVehicles.empty()) return false;

    // Get vehicle intentions
    auto vehicleIntention = getVehicleIntention(vehicleId, tJunctionId);
    
    for (int otherVehicleId : conflictingVehicles) {
        auto otherIntention = getVehicleIntention(otherVehicleId, tJunctionId);
        
        // Case 1: Both want opposite directions - first vehicle uses evasion
        if (vehicleIntention.targetDirection == getOppositeDirection(otherIntention.targetDirection)) {
            // First vehicle (lower ID) should use evasion route
            if (vehicleId < otherVehicleId) {
                std::cout << "Vehicle " << vehicleId << " using evasion route, other vehicle " << otherVehicleId << " has opposite direction" << std::endl;
                return tryEvasionRoute(vehicleId, tJunctionId);
            }
            return true; // Wait for other vehicle to use evasion or pass
        }
        
        // Case 2: Only one wants opposite direction - free route goes first
        if (otherIntention.targetDirection == getOppositeDirection(vehicleIntention.targetDirection)) {
            return true; // Wait for vehicle with free route
        }
        
        // Case 3: Both want same direction - first arrival goes first
        if (vehicleIntention.targetDirection == otherIntention.targetDirection) {
            // Compare arrival times or vehicle IDs
            if (vehicleId > otherVehicleId) {
                return true; // Other vehicle arrived first, wait
            }
        }
    }

    return false; // This vehicle can proceed
}

VehicleIntention VehicleController::getVehicleIntention(int vehicleId, int tJunctionId) {
    VehicleIntention intention;
    intention.targetDirection = Direction::UNKNOWN;

    const Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentPath.empty()) {
        return intention;
    }

    // Find where vehicle wants to go from T-junction
    int currentSegmentIndex = vehicle->currentSegmentIndex;
    if (currentSegmentIndex < vehicle->currentPath.size()) {
        int nextSegmentId = vehicle->currentPath[currentSegmentIndex];
        const PathSegment* segment = pathSystem->getSegment(nextSegmentId);
        if (segment) {
            // Determine direction based on segment endpoints
            const PathNode* tJunctionNode = pathSystem->getNode(tJunctionId);
            if (tJunctionNode) {
                int targetNodeId = (segment->startNodeId == tJunctionId) ? 
                                  segment->endNodeId : segment->startNodeId;
                const PathNode* targetNode = pathSystem->getNode(targetNodeId);
                if (targetNode) {
                    intention.targetDirection = calculateDirection(tJunctionNode->position, targetNode->position);
                }
            }
        }
    }

    return intention;
}

Direction VehicleController::calculateDirection(const Point& from, const Point& to) {
    Point diff = to - from;
    
    // Determine primary direction
    if (abs(diff.x) > abs(diff.y)) {
        return (diff.x > 0) ? Direction::EAST : Direction::WEST;
    } else {
        return (diff.y > 0) ? Direction::SOUTH : Direction::NORTH;
    }
}

Direction VehicleController::getOppositeDirection(Direction dir) {
    switch (dir) {
        case Direction::NORTH: return Direction::SOUTH;
        case Direction::SOUTH: return Direction::NORTH;
        case Direction::EAST: return Direction::WEST;
        case Direction::WEST: return Direction::EAST;
        default: return Direction::UNKNOWN;
    }
}

bool VehicleController::tryEvasionRoute(int vehicleId, int tJunctionId) {
    // Find available evasion segment
    int evasionSegmentId = segmentManager->findEvasionSegment(tJunctionId, -1);
    if (evasionSegmentId == -1) {
        return true; // No evasion available, must wait
    }

    // Check if evasion segment is free
    if (!segmentManager->canVehicleEnterSegment(evasionSegmentId, vehicleId)) {
        return true; // Evasion route blocked, must wait
    }

    // Modify vehicle path to use evasion route
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return true;

    std::cout << "Vehicle " << vehicleId << " taking evasion route via segment " << evasionSegmentId << std::endl;
    
    // Create new path with evasion
    std::vector<int> evasionPath = segmentManager->findEvasionRoute(tJunctionId, vehicle->targetNodeId, -1, vehicleId);
    if (!evasionPath.empty()) {
        vehicle->currentPath = evasionPath;
        vehicle->currentSegmentIndex = 0;
        return false; // Can proceed with evasion
    }

    return true; // Evasion failed, must wait
}

bool VehicleController::hasMinimumDistanceToOtherVehicles(const Auto& vehicle, float minDistance) {
    const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
    if (!currentNode) return true;
    
    // Relaxed distance checking for waiting nodes and junctions
    bool isAtWaitingNode = currentNode->isWaitingNode;
    bool isNearJunction = false;
    
    // Check if current node or nearby nodes are junctions
    if (segmentManager->isJunctionNode(vehicle.currentNodeId)) {
        isNearJunction = true;
    } else {
        // Check connected nodes
        for (int segmentId : currentNode->connectedSegments) {
            const PathSegment* segment = pathSystem->getSegment(segmentId);
            if (!segment) continue;
            
            int otherNodeId = (segment->startNodeId == vehicle.currentNodeId) ? 
                             segment->endNodeId : segment->startNodeId;
            if (segmentManager->isJunctionNode(otherNodeId)) {
                isNearJunction = true;
                break;
            }
        }
    }
    
    for (const Auto& otherVehicle : vehicles) {
        if (otherVehicle.vehicleId == vehicle.vehicleId) continue;
        
        float distance = vehicle.position.distanceTo(otherVehicle.position);
        
        // Apply different distance rules based on context
        float effectiveMinDistance = minDistance;
        
        if (isAtWaitingNode || isNearJunction) {
            // More lenient distance at waiting nodes and junctions to prevent deadlocks
            effectiveMinDistance = minDistance * 0.6f; // 60 pixels instead of 100
            
            // Check if other vehicle is also at waiting node or junction
            const PathNode* otherCurrentNode = pathSystem->getNode(otherVehicle.currentNodeId);
            if (otherCurrentNode && (otherCurrentNode->isWaitingNode || segmentManager->isJunctionNode(otherVehicle.currentNodeId))) {
                effectiveMinDistance = minDistance * 0.4f; // 40 pixels for both at special nodes
            }
        }
        
        if (distance < effectiveMinDistance) {
            return false;
        }
    }
    return true;
}

bool VehicleController::canMoveWithoutViolatingDistance(const Auto& vehicle, const Point& targetPosition, float minDistance) {
    const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
    if (!currentNode) return true;
    
    // Relaxed distance checking for waiting nodes and junctions
    bool isAtWaitingNode = currentNode->isWaitingNode;
    bool isNearJunction = segmentManager->isJunctionNode(vehicle.currentNodeId);
    
    for (const Auto& otherVehicle : vehicles) {
        if (otherVehicle.vehicleId == vehicle.vehicleId) continue;
        
        float distance = targetPosition.distanceTo(otherVehicle.position);
        
        // Apply different distance rules based on context
        float effectiveMinDistance = minDistance;
        
        if (isAtWaitingNode || isNearJunction) {
            // More lenient distance at waiting nodes and junctions
            effectiveMinDistance = minDistance * 0.6f; // 60 pixels instead of 100
            
            // Check if other vehicle is also at waiting node or junction
            const PathNode* otherCurrentNode = pathSystem->getNode(otherVehicle.currentNodeId);
            if (otherCurrentNode && (otherCurrentNode->isWaitingNode || segmentManager->isJunctionNode(otherVehicle.currentNodeId))) {
                effectiveMinDistance = minDistance * 0.4f; // 40 pixels for both at special nodes
            }
        }
        
        if (distance < effectiveMinDistance) {
            return false;
        }
    }
    return true;
}