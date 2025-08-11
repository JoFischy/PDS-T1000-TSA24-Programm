
#include "segment_manager.h"
#include <algorithm>
#include <iostream>

SegmentManager::SegmentManager(PathSystem* pathSys) : pathSystem(pathSys) {}

bool SegmentManager::canVehicleEnterSegment(int segmentId, int vehicleId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;
    
    // Vehicle can enter if segment is free or already occupied by this vehicle
    return !segment->isOccupied || segment->occupiedByVehicleId == vehicleId;
}

bool SegmentManager::reserveSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;
    
    // If already occupied by this vehicle, that's fine
    if (segment->isOccupied && segment->occupiedByVehicleId == vehicleId) {
        return true;
    }
    
    // If occupied by another vehicle, cannot reserve
    if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
        std::cout << "Segment " << segmentId << " is occupied by vehicle " 
                  << segment->occupiedByVehicleId << ", vehicle " << vehicleId << " must wait" << std::endl;
        return false;
    }
    
    // Segment is free, reserve it
    segment->isOccupied = true;
    segment->occupiedByVehicleId = vehicleId;
    vehicleSegmentMap[vehicleId] = segmentId;
    
    std::cout << "Vehicle " << vehicleId << " reserved segment " << segmentId << std::endl;
    return true;
}

void SegmentManager::releaseSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;
    
    // Only release if this vehicle actually occupies it
    if (segment->isOccupied && segment->occupiedByVehicleId == vehicleId) {
        segment->isOccupied = false;
        segment->occupiedByVehicleId = -1;
        vehicleSegmentMap.erase(vehicleId);
        
        std::cout << "Vehicle " << vehicleId << " released segment " << segmentId << std::endl;
        
        // Process any queued vehicles for this segment
        processQueue(segmentId);
    }
}

void SegmentManager::addToQueue(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;
    
    // Check if vehicle is already in queue
    auto& queue = segment->queuedVehicles;
    if (std::find(queue.begin(), queue.end(), vehicleId) == queue.end()) {
        queue.push_back(vehicleId);
        std::cout << "Vehicle " << vehicleId << " added to queue for segment " << segmentId << std::endl;
    }
}

void SegmentManager::removeFromQueue(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;
    
    auto& queue = segment->queuedVehicles;
    queue.erase(std::remove(queue.begin(), queue.end(), vehicleId), queue.end());
}

void SegmentManager::processQueue(int segmentId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment || segment->isOccupied || segment->queuedVehicles.empty()) return;
    
    // Try to assign segment to first vehicle in queue
    int nextVehicleId = segment->queuedVehicles.front();
    segment->queuedVehicles.erase(segment->queuedVehicles.begin());
    
    if (reserveSegment(segmentId, nextVehicleId)) {
        std::cout << "Segment " << segmentId << " assigned to queued vehicle " << nextVehicleId << std::endl;
    }
}

void SegmentManager::updateQueues() {
    // Process all segment queues
    for (const auto& segment : pathSystem->getSegments()) {
        if (!segment.isOccupied && !segment.queuedVehicles.empty()) {
            processQueue(segment.segmentId);
        }
    }
}

int SegmentManager::getVehicleSegment(int vehicleId) const {
    auto it = vehicleSegmentMap.find(vehicleId);
    return (it != vehicleSegmentMap.end()) ? it->second : -1;
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

    // Find a path considering current segment occupancy
    std::vector<int> blockedSegments;
    
    // Collect currently occupied segments (except by this vehicle)
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied && segment.occupiedByVehicleId != vehicleId) {
            blockedSegments.push_back(segment.segmentId);
        }
    }
    
    // Try to find path avoiding blocked segments
    std::vector<int> path = pathSystem->findPath(startNodeId, endNodeId, blockedSegments);
    
    // If no path found with blocked segments, try without restrictions
    if (path.empty()) {
        path = pathSystem->findPath(startNodeId, endNodeId, {});
    }
    
    return path;
}

std::vector<int> SegmentManager::findOptimalPath(int startNodeId, int endNodeId, int vehicleId) const {
    // Always return optimal path regardless of current occupancy
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

std::vector<int> SegmentManager::getOccupiedSegments() const {
    std::vector<int> occupied;
    
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied) {
            occupied.push_back(segment.segmentId);
        }
    }
    
    return occupied;
}

void SegmentManager::printSegmentStatus() const {
    std::cout << "=== Segment Status ===" << std::endl;
    for (const auto& segment : pathSystem->getSegments()) {
        std::cout << "Segment " << segment.segmentId << ": ";
        if (segment.isOccupied) {
            std::cout << "OCCUPIED by vehicle " << segment.occupiedByVehicleId;
        } else {
            std::cout << "FREE";
        }
        
        if (!segment.queuedVehicles.empty()) {
            std::cout << " (Queue: ";
            for (size_t i = 0; i < segment.queuedVehicles.size(); i++) {
                std::cout << segment.queuedVehicles[i];
                if (i < segment.queuedVehicles.size() - 1) std::cout << ", ";
            }
            std::cout << ")";
        }
        std::cout << std::endl;
    }
    std::cout << "===================" << std::endl;
}
