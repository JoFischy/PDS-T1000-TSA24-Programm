#include "segment_manager.h"
#include <algorithm>
#include <iostream>

SegmentManager::SegmentManager(PathSystem* pathSys) : pathSystem(pathSys) {}

bool SegmentManager::reserveSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;

    // If segment is occupied by the same vehicle, that's fine
    if (segment->isOccupied && segment->occupiedByVehicleId == vehicleId) {
        return true;
    }

    // If occupied by another vehicle, can't reserve
    if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
        // Add to queue if not already there
        if (!isVehicleInQueue(segmentId, vehicleId)) {
            addToQueue(segmentId, vehicleId);
        }
        return false;
    }

    // Segment is free - reserve it
    segment->isOccupied = true;
    segment->occupiedByVehicleId = vehicleId;
    setVehicleSegment(vehicleId, segmentId);

    std::cout << "Vehicle " << vehicleId << " reserved segment " << segmentId << std::endl;
    return true;
}

void SegmentManager::releaseSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;

    // Only release if actually occupied by this vehicle
    if (segment->isOccupied && segment->occupiedByVehicleId == vehicleId) {
        segment->isOccupied = false;
        segment->occupiedByVehicleId = -1;
        
        std::cout << "Vehicle " << vehicleId << " released segment " << segmentId << std::endl;
    }

    // Remove vehicle from tracking
    vehicleSegmentMap.erase(vehicleId);
    
    // Also remove from queue if present
    removeFromQueue(segmentId, vehicleId);
}

bool SegmentManager::isSegmentOccupied(int segmentId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    return segment ? segment->isOccupied : false;
}

int SegmentManager::getSegmentOccupant(int segmentId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    return (segment && segment->isOccupied) ? segment->occupiedByVehicleId : -1;
}

void SegmentManager::addToQueue(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;

    // Check if already in queue
    auto& queue = segment->queuedVehicles;
    if (std::find(queue.begin(), queue.end(), vehicleId) == queue.end()) {
        queue.push_back(vehicleId);
    }
}

void SegmentManager::removeFromQueue(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;

    auto& queue = segment->queuedVehicles;
    queue.erase(std::remove(queue.begin(), queue.end(), vehicleId), queue.end());
}

int SegmentManager::getNextInQueue(int segmentId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment || segment->queuedVehicles.empty()) return -1;

    return segment->queuedVehicles.front();
}

bool SegmentManager::isVehicleInQueue(int segmentId, int vehicleId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;

    const auto& queue = segment->queuedVehicles;
    return std::find(queue.begin(), queue.end(), vehicleId) != queue.end();
}

size_t SegmentManager::getQueueLength(int segmentId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    return segment ? segment->queuedVehicles.size() : 0;
}

void SegmentManager::setVehicleSegment(int vehicleId, int segmentId) {
    vehicleSegmentMap[vehicleId] = segmentId;
}

int SegmentManager::getVehicleSegment(int vehicleId) const {
    auto it = vehicleSegmentMap.find(vehicleId);
    return it != vehicleSegmentMap.end() ? it->second : -1;
}



std::vector<int> SegmentManager::getVehiclesReadyToMove() const {
    std::vector<int> readyVehicles;

    for (const auto& pair : vehicleSegmentMap) {
        int vehicleId = pair.first;
        int segmentId = pair.second;

        if (canVehicleEnterSegment(segmentId, vehicleId)) {
            readyVehicles.push_back(vehicleId);
        }
    }

    return readyVehicles;
}

void SegmentManager::removeVehicle(int vehicleId) {
    // Release any occupied segment
    int currentSegment = getVehicleSegment(vehicleId);
    if (currentSegment != -1) {
        releaseSegment(currentSegment, vehicleId);
    }

    // Remove from all queues
    for (const auto& segment : pathSystem->getSegments()) {
        removeFromQueue(segment.segmentId, vehicleId);
    }

    vehicleSegmentMap.erase(vehicleId);
}

std::vector<int> SegmentManager::findAvailablePath(int startNodeId, int endNodeId, int vehicleId) const {
    if (startNodeId == endNodeId) {
        return {};
    }

    // Always find a path - let the movement logic handle conflicts
    return pathSystem->findPath(startNodeId, endNodeId, {});
}

std::vector<int> SegmentManager::findOptimalPath(int startNodeId, int endNodeId, int vehicleId) const {
    // Normale Pfadfindung ohne Verkehrsausschluss für optimale Route
    return pathSystem->findPath(startNodeId, endNodeId, {});
}

bool SegmentManager::isPathClear(const std::vector<int>& path, int vehicleId) const {
    for (int segmentId : path) {
        if (!canVehicleEnterSegment(segmentId, vehicleId)) {
            return false;
        }
    }
    return true;
}

int SegmentManager::getSegmentCongestion(int segmentId) const {
    const PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return 0;

    int congestion = 0;
    if (segment->isOccupied) congestion++;
    congestion += segment->queuedVehicles.size();

    return congestion;
}

std::vector<int> SegmentManager::getAlternativePaths(int startNodeId, int endNodeId, int vehicleId) const {
    // Für diese einfache Implementierung geben wir nur den Standard-Pfad zurück
    // In einer erweiterten Version könnten hier multiple Routen berechnet werden
    return pathSystem->findPath(startNodeId, endNodeId, {});
}

bool SegmentManager::canVehicleEnterSegment(int segmentId, int vehicleId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;

    if (!segment->isOccupied) return true;

    // Can enter if already occupying this segment
    if (segment->occupiedByVehicleId == vehicleId) return true;
    
    // Allow shared access for segments with low congestion (experimental)
    // This allows multiple vehicles on longer segments
    if (segment->length > 100.0f && segment->queuedVehicles.size() < 2) {
        return true;
    }

    return false;
}

void SegmentManager::updateQueues() {
    // Process all segments and try to move queued vehicles
    for (const auto& segment : pathSystem->getSegments()) {
        if (!segment.isOccupied && !segment.queuedVehicles.empty()) {
            int nextVehicleId = segment.queuedVehicles.front();

            // Get the actual segment pointer for modification
            PathSegment* seg = pathSystem->getSegment(segment.segmentId);
            if (!seg) continue;

            // Double check it's not occupied
            if (!seg->isOccupied) {
                // Reserve the segment directly (bypass the queue check in reserveSegment)
                seg->isOccupied = true;
                seg->occupiedByVehicleId = nextVehicleId;
                setVehicleSegment(nextVehicleId, segment.segmentId);
                
                // Remove from queue since it now occupies the segment
                seg->queuedVehicles.erase(seg->queuedVehicles.begin());
                
                std::cout << "Queue processed: Vehicle " << nextVehicleId 
                         << " got segment " << segment.segmentId << std::endl;
            }
        }
    }
}