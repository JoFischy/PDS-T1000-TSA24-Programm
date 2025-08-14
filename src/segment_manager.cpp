
#include "segment_manager.h"
#include <iostream>
#include <algorithm>

SegmentManager::SegmentManager(PathSystem* pathSys) : pathSystem(pathSys) {
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
        
        // Prüfe ob Segment frei ist
        if (canVehicleEnterSegment(segmentId, -1)) {
            // Gib dem ersten Fahrzeug in der Warteschlange die Möglichkeit zu reservieren
            int waitingVehicleId = queue.front();
            queue.pop();
            
            if (reserveSegment(segmentId, waitingVehicleId)) {
                std::cout << "Fahrzeug " << waitingVehicleId << " aus Warteschlange für Segment " << segmentId << " bedient" << std::endl;
            }
        }
    }
}

void SegmentManager::removeVehicle(int vehicleId) {
    // Gib alle von diesem Fahrzeug besetzten Segmente frei
    auto it = vehicleToSegment.find(vehicleId);
    if (it != vehicleToSegment.end()) {
        releaseSegment(it->second, vehicleId);
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
