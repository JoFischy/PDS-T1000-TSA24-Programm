
#include "vehicle_controller.h"
#include <algorithm>
#include <iostream>
#include <random>

VehicleController::VehicleController(PathSystem* pathSys, SegmentManager* segmentMgr)
    : pathSystem(pathSys), segmentManager(segmentMgr), nextVehicleId(0),
      randomGenerator(std::random_device{}()), randomFloat(0.0f, 1.0f) {
}

int VehicleController::addVehicle(const Point& startPosition) {
    Auto vehicle(nextVehicleId, startPosition);
    
    // Finde nächsten Knoten als Startposition
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
    
    std::cout << "Vehicle " << nextVehicleId << " spawned at node " << nearestNodeId << std::endl;
    return nextVehicleId++;
}

void VehicleController::removeVehicle(int vehicleId) {
    auto it = vehicleIdToIndex.find(vehicleId);
    if (it == vehicleIdToIndex.end()) return;
    
    size_t index = it->second;
    
    // Gib alle reservierten Segmente frei
    segmentManager->removeVehicle(vehicleId);
    
    // Entferne aus Vector
    vehicles.erase(vehicles.begin() + index);
    vehicleIdToIndex.erase(it);
    
    // Update Indizes
    for (size_t i = index; i < vehicles.size(); i++) {
        vehicleIdToIndex[vehicles[i].vehicleId] = i;
    }
}

void VehicleController::spawnInitialVehicles() {
    if (!pathSystem || pathSystem->getNodeCount() < 4) {
        std::cerr << "Nicht genug Knoten für 4 Fahrzeuge" << std::endl;
        return;
    }
    
    const auto& nodes = pathSystem->getNodes();
    vehicles.clear();
    vehicleIdToIndex.clear();
    nextVehicleId = 0;
    
    // Spawne 4 Fahrzeuge an verschiedenen Knoten
    std::vector<int> spawnNodes = {0, 3, 6, 9}; // Verteilt über das Netzwerk
    
    for (int i = 0; i < 4 && i < spawnNodes.size() && spawnNodes[i] < nodes.size(); i++) {
        const PathNode& node = nodes[spawnNodes[i]];
        addVehicle(node.position);
        std::cout << "Fahrzeug " << (i + 1) << " spawned an Knoten " << node.nodeId << std::endl;
    }
    
    std::cout << "Spawned " << vehicles.size() << " Fahrzeuge" << std::endl;
}

void VehicleController::assignRandomTargetsToAllVehicles() {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;
    
    const auto& nodes = pathSystem->getNodes();
    std::uniform_int_distribution<> nodeDist(0, nodes.size() - 1);
    
    for (auto& vehicle : vehicles) {
        // Finde verschiedenes Ziel
        int targetNodeId;
        do {
            int randomIndex = nodeDist(randomGenerator);
            targetNodeId = nodes[randomIndex].nodeId;
        } while (targetNodeId == vehicle.currentNodeId && nodes.size() > 1);
        
        setVehicleTarget(vehicle.vehicleId, targetNodeId);
        std::cout << "Fahrzeug " << vehicle.vehicleId << " Ziel: Knoten " << targetNodeId << std::endl;
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

void VehicleController::setVehicleTargetNode(int vehicleId, int targetNodeId) {
    setVehicleTarget(vehicleId, targetNodeId);
}

void VehicleController::setVehicleTarget(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return;
    
    vehicle->targetNodeId = targetNodeId;
    vehicle->state = VehicleState::IDLE;
    
    std::cout << "Fahrzeug " << vehicleId << " neues Ziel: Knoten " << targetNodeId << std::endl;
}

void VehicleController::assignNewRandomTarget(int vehicleId) {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;
    
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;
    
    const auto& nodes = pathSystem->getNodes();
    std::uniform_int_distribution<> nodeDist(0, nodes.size() - 1);
    
    int targetNodeId;
    do {
        int randomIndex = nodeDist(randomGenerator);
        targetNodeId = nodes[randomIndex].nodeId;
    } while (targetNodeId == vehicle->currentNodeId && nodes.size() > 1);
    
    setVehicleTarget(vehicleId, targetNodeId);
}

void VehicleController::updateVehicles(float deltaTime) {
    for (Auto& vehicle : vehicles) {
        updateVehicleMovement(vehicle, deltaTime);
    }
    
    // Update SegmentManager
    segmentManager->updateQueues();
}

void VehicleController::updateVehicleMovement(Auto& vehicle, float deltaTime) {
    switch (vehicle.state) {
        case VehicleState::IDLE:
            handleIdleVehicle(vehicle);
            break;
            
        case VehicleState::MOVING:
            handleMovingVehicle(vehicle, deltaTime);
            break;
            
        case VehicleState::WAITING:
            handleWaitingVehicle(vehicle);
            break;
            
        case VehicleState::ARRIVED:
            // Weise neues Ziel nach kurzer Pause zu
            if (randomFloat(randomGenerator) < 0.01f) { // 1% Chance pro Frame
                assignNewRandomTarget(vehicle.vehicleId);
            }
            break;
    }
}

void VehicleController::handleIdleVehicle(Auto& vehicle) {
    if (vehicle.targetNodeId == -1) {
        vehicle.state = VehicleState::ARRIVED;
        return;
    }
    
    if (vehicle.currentNodeId == vehicle.targetNodeId) {
        vehicle.state = VehicleState::ARRIVED;
        std::cout << "Fahrzeug " << vehicle.vehicleId << " am Ziel angekommen" << std::endl;
        return;
    }
    
    // Plane Pfad zum Ziel
    if (planPath(vehicle.vehicleId, vehicle.targetNodeId)) {
        if (!vehicle.currentPath.empty()) {
            // Versuche erstes Segment zu reservieren
            int firstSegment = vehicle.currentPath[0];
            if (segmentManager->reserveSegment(firstSegment, vehicle.vehicleId)) {
                vehicle.state = VehicleState::MOVING;
                vehicle.currentSegmentIndex = 0;
                vehicle.progress = 0.0f;
                std::cout << "Fahrzeug " << vehicle.vehicleId << " startet Bewegung" << std::endl;
            } else {
                vehicle.state = VehicleState::WAITING;
                segmentManager->addToQueue(firstSegment, vehicle.vehicleId);
                std::cout << "Fahrzeug " << vehicle.vehicleId << " wartet auf Segment " << firstSegment << std::endl;
            }
        }
    }
}

void VehicleController::handleMovingVehicle(Auto& vehicle, float deltaTime) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        vehicle.state = VehicleState::ARRIVED;
        return;
    }
    
    moveVehicleAlongSegment(vehicle, deltaTime);
}

void VehicleController::handleWaitingVehicle(Auto& vehicle) {
    if (vehicle.currentPath.empty()) {
        vehicle.state = VehicleState::IDLE;
        return;
    }
    
    int nextSegment = vehicle.currentPath[vehicle.currentSegmentIndex];
    if (segmentManager->canVehicleEnterSegment(nextSegment, vehicle.vehicleId)) {
        if (segmentManager->reserveSegment(nextSegment, vehicle.vehicleId)) {
            vehicle.state = VehicleState::MOVING;
            vehicle.progress = 0.0f;
            std::cout << "Fahrzeug " << vehicle.vehicleId << " kann weiterfahren" << std::endl;
        }
    }
}

void VehicleController::moveVehicleAlongSegment(Auto& vehicle, float deltaTime) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        return;
    }
    
    int currentSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];
    const PathSegment* segment = pathSystem->getSegment(currentSegmentId);
    if (!segment) return;
    
    const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
    const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
    if (!startNode || !endNode) return;
    
    // Bestimme Richtung basierend auf aktuellem Knoten
    Point segmentStart, segmentEnd;
    int targetNodeId;
    
    if (vehicle.currentNodeId == segment->startNodeId) {
        segmentStart = startNode->position;
        segmentEnd = endNode->position;
        targetNodeId = segment->endNodeId;
    } else {
        segmentStart = endNode->position;
        segmentEnd = startNode->position;
        targetNodeId = segment->startNodeId;
    }
    
    // Bewege entlang Segment
    float segmentLength = segmentStart.distanceTo(segmentEnd);
    if (segmentLength > 0) {
        float moveSpeed = vehicle.speed * deltaTime;
        vehicle.progress += moveSpeed / segmentLength;
        
        if (vehicle.progress >= 1.0f) {
            // Segment komplett durchfahren
            vehicle.position = segmentEnd;
            vehicle.currentNodeId = targetNodeId;
            vehicle.progress = 0.0f;
            
            // Gib aktuelles Segment frei
            segmentManager->releaseSegment(currentSegmentId, vehicle.vehicleId);
            
            // Gehe zum nächsten Segment
            vehicle.currentSegmentIndex++;
            
            if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
                // Pfad beendet
                vehicle.state = VehicleState::ARRIVED;
                vehicle.currentPath.clear();
                std::cout << "Fahrzeug " << vehicle.vehicleId << " Pfad beendet" << std::endl;
            } else {
                // Versuche nächstes Segment zu reservieren
                int nextSegment = vehicle.currentPath[vehicle.currentSegmentIndex];
                if (segmentManager->reserveSegment(nextSegment, vehicle.vehicleId)) {
                    // Weiterfahren
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " wechselt zu Segment " << nextSegment << std::endl;
                } else {
                    vehicle.state = VehicleState::WAITING;
                    segmentManager->addToQueue(nextSegment, vehicle.vehicleId);
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " wartet auf nächstes Segment" << std::endl;
                }
            }
        } else {
            // Interpoliere Position entlang Segment
            vehicle.position = Point(
                segmentStart.x + (segmentEnd.x - segmentStart.x) * vehicle.progress,
                segmentStart.y + (segmentEnd.y - segmentStart.y) * vehicle.progress
            );
        }
    }
}

bool VehicleController::planPath(int vehicleId, int targetNodeId) {
    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return false;
    
    if (vehicle->currentNodeId == targetNodeId) {
        vehicle->currentPath.clear();
        return true;
    }
    
    std::vector<int> path = pathSystem->findPath(vehicle->currentNodeId, targetNodeId);
    
    if (!path.empty()) {
        vehicle->currentPath = path;
        vehicle->currentSegmentIndex = 0;
        vehicle->progress = 0.0f;
        std::cout << "Fahrzeug " << vehicleId << " Pfad geplant mit " << path.size() << " Segmenten" << std::endl;
        return true;
    }
    
    std::cout << "Fahrzeug " << vehicleId << " kein Pfad gefunden" << std::endl;
    return false;
}
