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

void VehicleController::setVehicleTargetNode(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;
    
    // Nur setzen wenn sich das Ziel geändert hat
    if (vehicle->targetNodeId != targetNodeId) {
        // First release any occupied segments
        releaseCurrentSegment(*vehicle);
        
        // Stelle sicher, dass das Fahrzeug an einem gültigen Knoten ist
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
        
        vehicle->targetNodeId = targetNodeId;
        vehicle->state = VehicleState::IDLE;
        clearPath(vehicleId);
        
        // Neue Route planen
        if (vehicle->currentNodeId != -1 && vehicle->currentNodeId != targetNodeId) {
            planPath(vehicleId, targetNodeId);
        }
        
        std::cout << "Vehicle " << vehicleId << " new target set to node " << targetNodeId << std::endl;
    }
}

void VehicleController::assignNewRandomTarget(int vehicleId) {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;
    
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;
    
    const auto& nodes = pathSystem->getNodes();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, nodes.size() - 1);
    
    // Finde einen anderen Zielknoten als den aktuellen
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
    
    // Stelle sicher, dass das Fahrzeug an einem gültigen Knoten ist
    if (vehicle->currentNodeId == -1) {
        // Finde nächstgelegenen Knoten
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
    
    // Bereits am Ziel?
    if (vehicle->currentNodeId == targetNodeId) {
        vehicle->state = VehicleState::ARRIVED;
        vehicle->currentPath.clear();
        return true;
    }
    
    // Finde immer einen Pfad, auch wenn Segmente belegt sind
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
    
    // Lösche aktuelle Route und plane neu
    clearPath(vehicleId);
    return planPath(vehicleId, vehicle->targetNodeId);
}

bool VehicleController::findAlternativePath(int vehicleId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1 || vehicle->targetNodeId == -1) return false;
    
    // Versuche mehrere Routen mit verschiedenen Strategien
    for (int attempt = 0; attempt < 3; attempt++) {
        std::vector<int> path = segmentManager->findAvailablePath(
            vehicle->currentNodeId, vehicle->targetNodeId, vehicleId);
        
        if (!path.empty()) {
            vehicle->currentPath = path;
            vehicle->currentSegmentIndex = 0;
            vehicle->state = VehicleState::MOVING;
            return true;
        }
        
        // Kurz warten zwischen Versuchen
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return false;
}

void VehicleController::handleBlockedVehicle(Auto& vehicle) {
    if (vehicle.state != VehicleState::WAITING) return;
    
    static std::unordered_map<int, float> waitTimers;
    static std::unordered_map<int, int> retryCount;
    
    waitTimers[vehicle.vehicleId] += 0.016f; // ca. 60 FPS
    
    // Warte 0.5 Sekunden bevor neuer Versuch (schneller)
    if (waitTimers[vehicle.vehicleId] > 0.5f) {
        waitTimers[vehicle.vehicleId] = 0.0f;
        retryCount[vehicle.vehicleId]++;
        
        // Versuche zuerst das aktuelle Segment erneut zu reservieren
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
        
        // Nach 5 Versuchen, plane komplett neue Route
        if (retryCount[vehicle.vehicleId] > 5) {
            retryCount[vehicle.vehicleId] = 0;
            std::cout << "Vehicle " << vehicle.vehicleId << " replanning path after timeout" << std::endl;
            clearPath(vehicle.vehicleId);
            planPath(vehicle.vehicleId, vehicle.targetNodeId);
        }
    }
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
    static int debugCounter = 0;
    bool debug = (debugCounter++ % 180 == 0); // Debug every 3 seconds at 60 FPS
    
    switch (vehicle.state) {
        case VehicleState::IDLE:
            if (debug) {
                std::cout << "Vehicle " << vehicle.vehicleId << " IDLE at node " 
                         << vehicle.currentNodeId << " target " << vehicle.targetNodeId << std::endl;
            }
            // Fahrzeug wartet auf neue Aufgabe oder nächstes Segment
            if (vehicle.targetNodeId != -1 && vehicle.currentNodeId != vehicle.targetNodeId) {
                if (!vehicle.currentPath.empty() && vehicle.currentSegmentIndex < vehicle.currentPath.size()) {
                    // Hat bereits einen Pfad - versuche Bewegung fortzusetzen
                    moveVehicleAlongPath(vehicle, deltaTime);
                } else {
                    // Neuen Pfad planen
                    if (debug) {
                        std::cout << "Vehicle " << vehicle.vehicleId << " planning new path" << std::endl;
                    }
                    planPath(vehicle.vehicleId, vehicle.targetNodeId);
                }
            }
            break;
            
        case VehicleState::MOVING:
            moveVehicleAlongPath(vehicle, deltaTime);
            break;
            
        case VehicleState::WAITING:
            // Behandle blockierte Fahrzeuge intelligent
            handleBlockedVehicle(vehicle);
            break;
            
        case VehicleState::ARRIVED:
            // Fahrzeug hat Ziel erreicht - automatisch neues Ziel zuweisen
            if (vehicle.currentNodeId == vehicle.targetNodeId) {
                if (debug) {
                    std::cout << "Vehicle " << vehicle.vehicleId << " arrived, assigning new target" << std::endl;
                }
                assignNewRandomTarget(vehicle.vehicleId);
            }
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
    
    // Get segment information
    const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
    if (!segment) {
        vehicle.state = VehicleState::WAITING;
        return;
    }
    
    // Only try to reserve segment when we're actually at one of the segment's nodes
    const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
    const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
    
    bool atSegmentStart = (vehicle.currentNodeId == startNode->nodeId);
    bool atSegmentEnd = (vehicle.currentNodeId == endNode->nodeId);
    bool atSegmentNode = atSegmentStart || atSegmentEnd;
    
    // Only check reservation when actually at segment boundary
    if (atSegmentNode && !segmentManager->canVehicleEnterSegment(currentSegmentId, vehicle.vehicleId)) {
        if (!segmentManager->reserveSegment(currentSegmentId, vehicle.vehicleId)) {
            vehicle.state = VehicleState::WAITING;
            std::cout << "Vehicle " << vehicle.vehicleId << " waiting for segment " << currentSegmentId << std::endl;
            return;
        }
    }
    
    // We can move on this segment
    vehicle.state = VehicleState::MOVING;
    
    // segment, startNode, and endNode are already defined above, no need to redeclare
    if (!segment) {
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
        vehicle.state = VehicleState::WAITING;
        return;
    }
    
    // Bestimme Zielknoten basierend auf der Pfadrichtung
    // startNode and endNode are already defined above
    if (!startNode || !endNode) return;
    
    // Bestimme den nächsten Knoten im Pfad
    Point targetPos;
    int targetNodeId;
    
    // Schaue voraus zum nächsten Segment um die Richtung zu bestimmen
    if (vehicle.currentSegmentIndex + 1 < vehicle.currentPath.size()) {
        int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex + 1];
        const PathSegment* nextSegment = pathSystem->getSegment(nextSegmentId);
        
        if (nextSegment) {
            // Finde den gemeinsamen Knoten zwischen aktuellem und nächstem Segment
            int sharedNode = -1;
            if (segment->startNodeId == nextSegment->startNodeId || segment->startNodeId == nextSegment->endNodeId) {
                sharedNode = segment->startNodeId;
            } else if (segment->endNodeId == nextSegment->startNodeId || segment->endNodeId == nextSegment->endNodeId) {
                sharedNode = segment->endNodeId;
            }
            
            if (sharedNode != -1) {
                // Fahre zum gemeinsamen Knoten
                const PathNode* sharedNodePtr = pathSystem->getNode(sharedNode);
                if (sharedNodePtr) {
                    targetPos = sharedNodePtr->position;
                    targetNodeId = sharedNode;
                }
            } else {
                // Fallback: fahre zum nächstgelegenen Ende
                float distToStart = vehicle.position.distanceTo(startNode->position);
                float distToEnd = vehicle.position.distanceTo(endNode->position);
                if (distToStart < distToEnd) {
                    targetPos = startNode->position;
                    targetNodeId = startNode->nodeId;
                } else {
                    targetPos = endNode->position;
                    targetNodeId = endNode->nodeId;
                }
            }
        }
    } else {
        // Letztes Segment - fahre zum Zielknoten
        if (vehicle.targetNodeId == startNode->nodeId) {
            targetPos = startNode->position;
            targetNodeId = startNode->nodeId;
        } else if (vehicle.targetNodeId == endNode->nodeId) {
            targetPos = endNode->position;
            targetNodeId = endNode->nodeId;
        } else {
            // Fahre zum nächstgelegenen Ende
            float distToStart = vehicle.position.distanceTo(startNode->position);
            float distToEnd = vehicle.position.distanceTo(endNode->position);
            if (distToStart < distToEnd) {
                targetPos = startNode->position;
                targetNodeId = startNode->nodeId;
            } else {
                targetPos = endNode->position;
                targetNodeId = endNode->nodeId;
            }
        }
    }
    
    vehicle.targetPosition = targetPos;
    
    // Berechne Bewegung mit verbesserter Geschwindigkeit
    float moveDistance = vehicle.speed * deltaTime;
    Point direction = (targetPos - vehicle.position).normalize();
    float distanceToTarget = vehicle.position.distanceTo(targetPos);
    
    // Debug output für Bewegung
    static int debugCounter = 0;
    if (debugCounter++ % 60 == 0) { // Alle 60 Frames (ca. 1 Sekunde)
        std::cout << "Vehicle " << vehicle.vehicleId << " moving: distance=" << distanceToTarget 
                  << ", speed=" << vehicle.speed << ", moveDistance=" << moveDistance << std::endl;
    }
    
    if (moveDistance >= distanceToTarget || distanceToTarget < 10.0f) {
        // Am Zielknoten des Segments angekommen
        vehicle.position = targetPos;
        vehicle.currentNodeId = targetNodeId;
        
        // Segment sofort freigeben wenn wir den Zielknoten erreicht haben
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
        
        // Zum nächsten Segment wechseln
        vehicle.currentSegmentIndex++;
        
        if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
            // Gesamtes Ziel erreicht
            vehicle.state = VehicleState::ARRIVED;
            vehicle.currentPath.clear();
            std::cout << "Vehicle " << vehicle.vehicleId << " arrived at final target node " << vehicle.targetNodeId << std::endl;
        } else {
            // Zurück zu IDLE für nächstes Segment
            vehicle.state = VehicleState::IDLE;
            std::cout << "Vehicle " << vehicle.vehicleId << " reached node " << targetNodeId 
                      << ", preparing for next segment " << vehicle.currentSegmentIndex << std::endl;
        }
    } else {
        // Weiter in Richtung Zielknoten bewegen
        Point newPosition = vehicle.position + direction * moveDistance;
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