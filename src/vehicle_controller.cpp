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
    std::vector<int> validTargets;

    // Sammle alle Knoten, die keine Warteknoten sind
    for (const auto& node : nodes) {
        if (!node.isWaitingNode) {
            validTargets.push_back(node.nodeId);
        }
    }

    if (validTargets.empty()) return;

    std::uniform_int_distribution<> targetDist(0, validTargets.size() - 1);

    for (auto& vehicle : vehicles) {
        // Finde verschiedenes Ziel
        int targetNodeId;
        do {
            int randomIndex = targetDist(randomGenerator);
            targetNodeId = validTargets[randomIndex];
        } while (targetNodeId == vehicle.currentNodeId && validTargets.size() > 1);

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
    vehicle->state = VehicleState::IDLE; // Start im Idle-Zustand, um Pfadplanung zu ermöglichen
    vehicle->waitingForSegment = -1;
    vehicle->isAtWaitingNode = false;

    std::cout << "Fahrzeug " << vehicleId << " neues Ziel: Knoten " << targetNodeId << std::endl;
}

void VehicleController::assignNewRandomTarget(int vehicleId) {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return;

    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;

    const auto& nodes = pathSystem->getNodes();
    std::vector<int> validTargets;

    // Sammle alle Knoten, die keine Warteknoten sind
    for (const auto& node : nodes) {
        if (!node.isWaitingNode && node.nodeId != vehicle->currentNodeId) {
            validTargets.push_back(node.nodeId);
        }
    }

    if (validTargets.empty()) return;

    std::uniform_int_distribution<> targetDist(0, validTargets.size() - 1);
    int randomIndex = targetDist(randomGenerator);
    int targetNodeId = validTargets[randomIndex];

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
            // Weise sofort neues Ziel zu, wenn am Ziel angekommen
            assignNewRandomTarget(vehicle.vehicleId);
            break;
    }
}

void VehicleController::handleIdleVehicle(Auto& vehicle) {
    if (vehicle.currentPath.empty()) {
        // Generiere neuen zufälligen Zielknoten
        int newTarget = generateRandomTarget(vehicle.vehicleId);
        if (newTarget != -1 && planPath(vehicle.vehicleId, newTarget)) {
            std::cout << "Fahrzeug " << vehicle.vehicleId << " neuer Pfad geplant zu Knoten " << newTarget << std::endl;
        } else {
            std::cout << "Fahrzeug " << vehicle.vehicleId << " konnte keinen Pfad planen" << std::endl;
            vehicle.state = VehicleState::ARRIVED;
            return;
        }
    }

    // Prüfe ob wir an einem Warteknoten sind
    const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
    if (currentNode && currentNode->isWaitingNode) {
        // An Warteknoten: Bestimme die nächste Section
        if (vehicle.currentSegmentIndex < vehicle.currentPath.size()) {
            int nextSegmentId = vehicle.currentPath[vehicle.currentSegmentIndex];

            // Baue Section basierend auf dem nächsten Segment
            std::vector<int> nextSection = segmentManager->buildSectionFromSegment(nextSegmentId);

            std::cout << "Fahrzeug " << vehicle.vehicleId << " an Warteknoten " << vehicle.currentNodeId 
                      << " prüft Section mit " << nextSection.size() << " Segmenten (Segmente: ";
            for (int seg : nextSection) std::cout << seg << " ";
            std::cout << ")" << std::endl;

            if (!nextSection.empty()) {
                if (segmentManager->canVehicleEnterSection(nextSection, vehicle.vehicleId)) {
                    if (segmentManager->reserveSection(nextSection, vehicle.vehicleId)) {
                        vehicle.state = VehicleState::MOVING;
                        vehicle.progress = 0.0f;
                        vehicle.isAtWaitingNode = false;
                        std::cout << "Fahrzeug " << vehicle.vehicleId << " Section ERFOLGREICH reserviert!" << std::endl;
                    } else {
                        std::cout << "Fahrzeug " << vehicle.vehicleId << " Section-Reservierung FEHLGESCHLAGEN" << std::endl;
                        vehicle.state = VehicleState::WAITING;
                        vehicle.waitingForSegment = nextSegmentId;
                    }
                } else {
                    // Section blockiert - in Warteschlange einreihen
                    segmentManager->addToQueue(nextSegmentId, vehicle.vehicleId);
                    vehicle.state = VehicleState::WAITING;
                    vehicle.waitingForSegment = nextSegmentId;
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " Section BLOCKIERT, wechselt zu WAITING" << std::endl;
                }
            } else {
                std::cout << "Fahrzeug " << vehicle.vehicleId << " LEERE Section gefunden!" << std::endl;
                vehicle.state = VehicleState::ARRIVED;
            }
        } else {
            vehicle.state = VehicleState::ARRIVED;
            std::cout << "Fahrzeug " << vehicle.vehicleId << " IDLE an Warteknoten mit leerem Pfad, wechselt zu ARRIVED." << std::endl;
        }
    } else {
        // Nicht an Warteknoten - direkt weiterfahren
        if (!vehicle.currentPath.empty() && vehicle.currentNodeId != vehicle.targetNodeId) {
            vehicle.state = VehicleState::MOVING;
            vehicle.progress = 0.0f;
            vehicle.isAtWaitingNode = false;
            std::cout << "Fahrzeug " << vehicle.vehicleId << " nicht an Warteknoten, fährt direkt weiter." << std::endl;
        } else if (vehicle.currentNodeId == vehicle.targetNodeId) {
            vehicle.state = VehicleState::ARRIVED;
            std::cout << "Fahrzeug " << vehicle.vehicleId << " bereits am Ziel." << std::endl;
        }
    }
}

void VehicleController::handleMovingVehicle(Auto& vehicle, float deltaTime) {
    if (vehicle.currentPath.empty() || 
        vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
        // Ungültiger Zustand, zurück zu IDLE oder ARRIVED
        if(vehicle.currentNodeId == vehicle.targetNodeId) {
            vehicle.state = VehicleState::ARRIVED;
        } else {
            vehicle.state = VehicleState::IDLE;
        }
        return;
    }

    moveVehicleAlongSegment(vehicle, deltaTime);
}

void VehicleController::handleWaitingVehicle(Auto& vehicle) {
    if (vehicle.currentPath.empty() || vehicle.waitingForSegment == -1) {
        vehicle.state = VehicleState::IDLE; // Zurück zu IDLE, wenn Wartebedingung nicht mehr erfüllt
        return;
    }

    // Prüfe ob wir an einem Warteknoten sind
    const PathNode* currentNode = pathSystem->getNode(vehicle.currentNodeId);
    if (!currentNode || !currentNode->isWaitingNode) {
        // Nicht an Warteknoten - zurück zu IDLE
        vehicle.state = VehicleState::IDLE;
        vehicle.waitingForSegment = -1;
        return;
    }

    // Baue die Section vom wartenden Segment aus
    std::vector<int> sectionSegments = segmentManager->buildSectionFromSegment(vehicle.waitingForSegment);

    // Prüfe ob Section jetzt verfügbar ist
    if (segmentManager->canVehicleEnterSection(sectionSegments, vehicle.vehicleId)) {
        // Reserviere Section und fahre weiter
        if (segmentManager->reserveSection(sectionSegments, vehicle.vehicleId)) {
            vehicle.state = VehicleState::MOVING;
            vehicle.progress = 0.0f; // Fortschritt zurücksetzen
            vehicle.isAtWaitingNode = false; // Nicht mehr am Warteknoten für diese Section
            vehicle.waitingForSegment = -1; // Wartebedingung erfüllt
            segmentManager->removeFromQueue(vehicle.waitingForSegment, vehicle.vehicleId); // Aus Warteschlange entfernen
            std::cout << "Fahrzeug " << vehicle.vehicleId << " kann von Warteknoten weiterfahren" << std::endl;
        }
    } else {
        // Section noch belegt - bleibe in Warteschlange
        std::cout << "Fahrzeug " << vehicle.vehicleId << " wartet weiterhin an Warteknoten" << std::endl;
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
            vehicle.progress = 0.0f; // Fortschritt für nächstes Segment zurücksetzen

            // Aktualisiere Segmentindex vor der Knotenprüfung
            vehicle.currentSegmentIndex++;

            // Prüfe ob wir einen Warteknoten erreicht haben
            const PathNode* reachedNode = pathSystem->getNode(targetNodeId);
            if (reachedNode && reachedNode->isWaitingNode) {
                // An Warteknoten angekommen - gib aktuelle Section frei
                std::vector<int> currentSection = segmentManager->getVehicleSection(vehicle.vehicleId);
                if (!currentSection.empty()) {
                    segmentManager->releaseSection(currentSection, vehicle.vehicleId);
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " gibt Section an Warteknoten " << targetNodeId << " frei" << std::endl;
                }

                // Prüfe ob Pfad beendet ist
                if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
                    vehicle.state = VehicleState::ARRIVED;
                    vehicle.currentPath.clear();
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " Pfad beendet an Warteknoten" << std::endl;
                } else {
                    // An Warteknoten - wechsle zu IDLE für neue Section-Reservierung
                    vehicle.state = VehicleState::IDLE;
                    vehicle.isAtWaitingNode = true;
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " an Warteknoten " << targetNodeId << " - bereit für nächste Section" << std::endl;
                }
            } else {
                // T-Section oder normaler Knoten erreicht - fahre weiter in derselben Section
                if (vehicle.currentSegmentIndex >= vehicle.currentPath.size()) {
                    // Pfad beendet - gib Section frei
                    std::vector<int> currentSection = segmentManager->getVehicleSection(vehicle.vehicleId);
                    if (!currentSection.empty()) {
                        segmentManager->releaseSection(currentSection, vehicle.vehicleId);
                        std::cout << "Fahrzeug " << vehicle.vehicleId << " gibt Section am Ziel frei" << std::endl;
                    }
                    vehicle.state = VehicleState::ARRIVED;
                    vehicle.currentPath.clear();
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " Pfad beendet" << std::endl;
                } else {
                    std::cout << "Fahrzeug " << vehicle.vehicleId << " passiert Knoten " << targetNodeId << " in derselben Section" << std::endl;
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

int VehicleController::generateRandomTarget(int vehicleId) {
    if (!pathSystem || pathSystem->getNodeCount() < 2) return -1;

    Auto* vehicle = getVehicle(vehicleId);
    if (!vehicle) return -1;

    const auto& nodes = pathSystem->getNodes();
    std::vector<int> validTargets;

    // Sammle alle Knoten, die keine Warteknoten sind
    for (const auto& node : nodes) {
        if (!node.isWaitingNode && node.nodeId != vehicle->currentNodeId) {
            validTargets.push_back(node.nodeId);
        }
    }

    if (validTargets.empty()) return -1;

    std::uniform_int_distribution<> targetDist(0, validTargets.size() - 1);
    int randomIndex = targetDist(randomGenerator);

    return validTargets[randomIndex];
}