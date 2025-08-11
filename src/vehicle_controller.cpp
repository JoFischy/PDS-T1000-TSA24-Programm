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
    
    // Bereits am Ziel?
    if (vehicle->currentNodeId == targetNodeId) {
        vehicle->state = VehicleState::ARRIVED;
        vehicle->currentPath.clear();
        return true;
    }
    
    // Versuche verfügbare Route zu finden
    std::vector<int> path = segmentManager->findAvailablePath(
        vehicle->currentNodeId, targetNodeId, vehicleId);
    
    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->targetNodeId = targetNodeId;
        vehicle->state = VehicleState::MOVING;
        std::cout << "Vehicle " << vehicleId << " planned path with " << path.size() << " segments" << std::endl;
        return true;
    } else {
        // Keine direkte Route verfügbar - warten oder alternative Route finden
        vehicle->state = VehicleState::WAITING;
        std::cout << "Vehicle " << vehicleId << " waiting - no available path to target" << std::endl;
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
    
    // Versuche alternative Route zu finden
    if (findAlternativePath(vehicle.vehicleId)) {
        std::cout << "Vehicle " << vehicle.vehicleId << " found alternative path" << std::endl;
    } else {
        // Bleibe im Wartezustand und versuche es später erneut
        static std::unordered_map<int, float> waitTimers;
        waitTimers[vehicle.vehicleId] += 0.016f; // ca. 60 FPS
        
        if (waitTimers[vehicle.vehicleId] > 2.0f) { // Alle 2 Sekunden neu versuchen
            waitTimers[vehicle.vehicleId] = 0.0f;
            replanPathIfBlocked(vehicle.vehicleId);
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
    switch (vehicle.state) {
        case VehicleState::IDLE:
            // Fahrzeug wartet auf neue Aufgabe
            if (vehicle.targetNodeId != -1 && vehicle.currentNodeId != vehicle.targetNodeId) {
                planPath(vehicle.vehicleId, vehicle.targetNodeId);
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
    
    // Reserviere das aktuelle Segment wenn noch nicht geschehen
    if (!segmentManager->reserveSegment(currentSegmentId, vehicle.vehicleId)) {
        vehicle.state = VehicleState::WAITING;
        std::cout << "Vehicle " << vehicle.vehicleId << " waiting for segment " << currentSegmentId << std::endl;
        return;
    }
    
    const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
    if (!segment) {
        vehicle.state = VehicleState::WAITING;
        return;
    }
    
    // Ermittle Start- und Endpositionen des Segments
    const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
    const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
    if (!startNode || !endNode) return;
    
    Point segmentStart = startNode->position;
    Point segmentEnd = endNode->position;
    
    // Bestimme Fahrtrichtung: Fahrzeug soll zum Zielknoten des Segments fahren
    // Der Zielknoten ist derjenige, der NICHT der aktuelle Knoten des Fahrzeugs ist
    if (vehicle.currentNodeId == startNode->nodeId) {
        // Fahrzeug ist am Startknoten, fahre zum Endknoten
        vehicle.targetPosition = segmentEnd;
    } else {
        // Fahrzeug ist am Endknoten oder irgendwo anders, fahre zum Startknoten
        std::swap(segmentStart, segmentEnd);
        std::swap(startNode, endNode);
        vehicle.targetPosition = segmentEnd;
    }
    
    // Berechne Bewegung zum Zielknoten
    Point targetPos = (vehicle.currentNodeId == startNode->nodeId) ? segmentEnd : segmentStart;
    const PathNode* targetNodePtr = (vehicle.currentNodeId == startNode->nodeId) ? endNode : startNode;
    
    vehicle.targetPosition = targetPos;
    
    float distance = vehicle.speed * deltaTime;
    Point direction = (targetPos - vehicle.position).normalize();
    
    // Prüfe ob wir das Segmentende erreicht haben
    float distanceToTarget = vehicle.position.distanceTo(targetPos);
    
    if (distance >= distanceToTarget) {
        // Am Ziel des Segments angekommen
        vehicle.position = targetPos;
        vehicle.currentNodeId = targetNodePtr->nodeId;
        
        // Segment freigeben
        segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
        
        // Zum nächsten Segment wechseln
        vehicle.currentSegmentIndex++;
        
        if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
            // Ziel erreicht
            vehicle.state = VehicleState::ARRIVED;
            vehicle.currentPath.clear();
            std::cout << "Vehicle " << vehicle.vehicleId << " arrived at target node " << vehicle.targetNodeId << std::endl;
        }
    } else {
        // Weiter zum Zielknoten bewegen
        vehicle.position = vehicle.position + direction * distance;
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