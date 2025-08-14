
#include "segment_manager.h"
#include <iostream>
#include <algorithm>
#include <set>

SegmentManager::SegmentManager(PathSystem* pathSys) : pathSystem(pathSys) {
}

bool SegmentManager::reserveSection(const std::vector<int>& sectionSegments, int vehicleId) {
    if (!pathSystem || sectionSegments.empty()) return false;
    
    // Prüfe ob alle Segmente der Section frei sind
    for (int segmentId : sectionSegments) {
        if (segmentOccupancy.find(segmentId) != segmentOccupancy.end()) {
            return false; // Mindestens ein Segment bereits besetzt
        }
    }
    
    // Reserviere alle Segmente der Section
    for (int segmentId : sectionSegments) {
        PathSegment* segment = pathSystem->getSegment(segmentId);
        if (segment) {
            segmentOccupancy[segmentId] = vehicleId;
            segment->isOccupied = true;
            segment->occupiedByVehicleId = vehicleId;
        }
    }
    
    vehicleToSection[vehicleId] = sectionSegments;
    std::cout << "Section mit " << sectionSegments.size() << " Segmenten reserviert für Fahrzeug " << vehicleId << std::endl;
    return true;
}

bool SegmentManager::reserveSegment(int segmentId, int vehicleId) {
    if (!pathSystem) return false;
    
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;
    
    // Prüfe ob Segment frei ist
    if (segmentOccupancy.find(segmentId) != segmentOccupancy.end()) {
        return false; // Bereits besetzt
    }
    
    // Reserviere Segment
    segmentOccupancy[segmentId] = vehicleId;
    vehicleToSegment[vehicleId] = segmentId;
    segment->isOccupied = true;
    segment->occupiedByVehicleId = vehicleId;
    
    std::cout << "Segment " << segmentId << " reserviert für Fahrzeug " << vehicleId << std::endl;
    return true;
}

void SegmentManager::releaseSection(const std::vector<int>& sectionSegments, int vehicleId) {
    if (!pathSystem) return;
    
    // Gib alle Segmente der Section frei
    for (int segmentId : sectionSegments) {
        PathSegment* segment = pathSystem->getSegment(segmentId);
        if (segment && segmentOccupancy.find(segmentId) != segmentOccupancy.end() 
            && segmentOccupancy[segmentId] == vehicleId) {
            segmentOccupancy.erase(segmentId);
            segment->isOccupied = false;
            segment->occupiedByVehicleId = -1;
        }
    }
    
    vehicleToSection.erase(vehicleId);
    std::cout << "Section mit " << sectionSegments.size() << " Segmenten freigegeben von Fahrzeug " << vehicleId << std::endl;
}

void SegmentManager::releaseSegment(int segmentId, int vehicleId) {
    if (!pathSystem) return;
    
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;
    
    // Prüfe ob Fahrzeug das Segment besitzt
    auto it = segmentOccupancy.find(segmentId);
    if (it == segmentOccupancy.end() || it->second != vehicleId) {
        return; // Fahrzeug besitzt das Segment nicht
    }
    
    // Gib Segment frei
    segmentOccupancy.erase(segmentId);
    vehicleToSegment.erase(vehicleId);
    segment->isOccupied = false;
    segment->occupiedByVehicleId = -1;
    
    std::cout << "Segment " << segmentId << " freigegeben von Fahrzeug " << vehicleId << std::endl;
}

bool SegmentManager::canVehicleEnterSegment(int segmentId, int vehicleId) const {
    // Segment ist frei oder bereits von diesem Fahrzeug besetzt
    auto it = segmentOccupancy.find(segmentId);
    return (it == segmentOccupancy.end() || it->second == vehicleId);
}

void SegmentManager::addToQueue(int segmentId, int vehicleId) {
    // Prüfe ob Fahrzeug bereits in der Warteschlange ist
    auto& queue = segmentQueues[segmentId];
    std::queue<int> tempQueue = queue;
    bool alreadyInQueue = false;
    
    while (!tempQueue.empty()) {
        if (tempQueue.front() == vehicleId) {
            alreadyInQueue = true;
            break;
        }
        tempQueue.pop();
    }
    
    if (!alreadyInQueue) {
        queue.push(vehicleId);
        std::cout << "Fahrzeug " << vehicleId << " zur Warteschlange für Segment " << segmentId << " hinzugefügt" << std::endl;
    }
}

void SegmentManager::removeFromQueue(int segmentId, int vehicleId) {
    auto it = segmentQueues.find(segmentId);
    if (it == segmentQueues.end()) return;
    
    std::queue<int>& queue = it->second;
    std::queue<int> newQueue;
    
    while (!queue.empty()) {
        int id = queue.front();
        queue.pop();
        if (id != vehicleId) {
            newQueue.push(id);
        }
    }
    
    it->second = newQueue;
}

void SegmentManager::updateQueues() {
    for (auto& pair : segmentQueues) {
        int segmentId = pair.first;
        std::queue<int>& queue = pair.second;
        
        if (queue.empty()) continue;
        
        // Baue Section vom wartenden Segment aus
        std::vector<int> sectionSegments = buildSectionFromSegment(segmentId);
        
        // Prüfe ob gesamte Section frei ist
        if (canVehicleEnterSection(sectionSegments, -1)) {
            // Gib dem ersten Fahrzeug in der Warteschlange die Möglichkeit die Section zu reservieren
            int waitingVehicleId = queue.front();
            queue.pop();
            
            if (reserveSection(sectionSegments, waitingVehicleId)) {
                std::cout << "Fahrzeug " << waitingVehicleId << " aus Warteschlange bedient - Section mit " << sectionSegments.size() << " Segmenten reserviert" << std::endl;
            }
        }
    }
}

bool SegmentManager::canVehicleEnterSection(const std::vector<int>& sectionSegments, int vehicleId) const {
    for (int segmentId : sectionSegments) {
        auto it = segmentOccupancy.find(segmentId);
        if (it != segmentOccupancy.end() && it->second != vehicleId) {
            return false; // Mindestens ein Segment ist von anderem Fahrzeug besetzt
        }
    }
    return true;
}

std::vector<int> SegmentManager::getSectionSegments(int startNodeId, int endNodeId) const {
    if (!pathSystem) return {};
    
    // Verwende Pathfinding um alle Segmente zwischen zwei T-Sections zu finden
    std::vector<int> path = pathSystem->findPath(startNodeId, endNodeId);
    return path;
}

std::vector<int> SegmentManager::getVehicleSection(int vehicleId) const {
    auto it = vehicleToSection.find(vehicleId);
    return (it != vehicleToSection.end()) ? it->second : std::vector<int>();
}

void SegmentManager::removeVehicle(int vehicleId) {
    // Gib alle von diesem Fahrzeug besetzten Segmente frei
    auto sectionIt = vehicleToSection.find(vehicleId);
    if (sectionIt != vehicleToSection.end()) {
        releaseSection(sectionIt->second, vehicleId);
    }
    
    auto segmentIt = vehicleToSegment.find(vehicleId);
    if (segmentIt != vehicleToSegment.end()) {
        releaseSegment(segmentIt->second, vehicleId);
    }
    
    // Entferne aus allen Warteschlangen
    for (auto& pair : segmentQueues) {
        removeFromQueue(pair.first, vehicleId);
    }
}

int SegmentManager::getVehicleSegment(int vehicleId) const {
    auto it = vehicleToSegment.find(vehicleId);
    return (it != vehicleToSegment.end()) ? it->second : -1;
}

bool SegmentManager::isSegmentOccupied(int segmentId) const {
    return segmentOccupancy.find(segmentId) != segmentOccupancy.end();
}

int SegmentManager::getSegmentOccupant(int segmentId) const {
    auto it = segmentOccupancy.find(segmentId);
    return (it != segmentOccupancy.end()) ? it->second : -1;
}

std::vector<int> SegmentManager::buildSectionFromSegment(int startSegmentId) const {
    if (!pathSystem) return {};
    
    const PathSegment* startSegment = pathSystem->getSegment(startSegmentId);
    if (!startSegment) return {};
    
    // Finde die beiden T-Sectionen zwischen denen das Segment liegt
    int startTSection = -1;
    int endTSection = -1;
    
    // Prüfe beide Enden des Startsegments
    const PathNode* node1 = pathSystem->getNode(startSegment->startNodeId);
    const PathNode* node2 = pathSystem->getNode(startSegment->endNodeId);
    
    if (node1 && (node1->connectedSegments.size() >= 3 || node1->isWaitingNode)) {
        startTSection = node1->nodeId;
    }
    if (node2 && (node2->connectedSegments.size() >= 3 || node2->isWaitingNode)) {
        endTSection = node2->nodeId;
    }
    
    // Wenn noch keine T-Sections gefunden, suche in beide Richtungen
    if (startTSection == -1) {
        startTSection = findNearestTSection(startSegment->startNodeId, startSegmentId);
    }
    if (endTSection == -1) {
        endTSection = findNearestTSection(startSegment->endNodeId, startSegmentId);
    }
    
    if (startTSection == -1 || endTSection == -1) {
        std::cout << "Keine T-Sections für Segment " << startSegmentId << " gefunden" << std::endl;
        return {startSegmentId}; // Nur das eine Segment
    }
    
    // Finde alle Segmente zwischen den T-Sections
    std::vector<int> path = pathSystem->findPath(startTSection, endTSection);
    
    std::cout << "Section zwischen T-Sections " << startTSection << " und " << endTSection 
              << " mit " << path.size() << " Segmenten gefunden" << std::endl;
    
    return path;
}

int SegmentManager::findNearestTSection(int startNodeId, int excludeSegmentId) const {
    std::queue<int> queue;
    std::set<int> visited;
    std::set<int> visitedSegments;
    
    queue.push(startNodeId);
    visited.insert(startNodeId);
    
    while (!queue.empty()) {
        int currentNodeId = queue.front();
        queue.pop();
        
        const PathNode* currentNode = pathSystem->getNode(currentNodeId);
        if (!currentNode) continue;
        
        // Prüfe ob aktueller Knoten eine T-Section ist
        if (currentNodeId != startNodeId && 
            (currentNode->connectedSegments.size() >= 3 || currentNode->isWaitingNode)) {
            return currentNodeId;
        }
        
        // Füge benachbarte Knoten hinzu
        for (int segmentId : currentNode->connectedSegments) {
            if (segmentId == excludeSegmentId || visitedSegments.count(segmentId)) continue;
            
            visitedSegments.insert(segmentId);
            const PathSegment* segment = pathSystem->getSegment(segmentId);
            if (!segment) continue;
            
            int nextNodeId = (segment->startNodeId == currentNodeId) ? 
                             segment->endNodeId : segment->startNodeId;
            
            if (!visited.count(nextNodeId)) {
                visited.insert(nextNodeId);
                queue.push(nextNodeId);
            }
        }
    }
    
    return -1; // Keine T-Section gefunden
}
