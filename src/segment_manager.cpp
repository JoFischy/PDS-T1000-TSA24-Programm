
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
    } else {
        // If reservation failed, put vehicle back at front of queue
        segment->queuedVehicles.insert(segment->queuedVehicles.begin(), nextVehicleId);
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

float SegmentManager::estimatePathTime(const std::vector<int>& path, int vehicleId) const {
    if (path.empty()) return 0.0f;
    
    float totalTime = 0.0f;
    const float vehicleSpeed = 100.0f; // Default vehicle speed
    
    for (int segmentId : path) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        const PathNode* startNode = pathSystem->getNode(segment->startNodeId);
        const PathNode* endNode = pathSystem->getNode(segment->endNodeId);
        if (!startNode || !endNode) continue;
        
        float distance = startNode->position.distanceTo(endNode->position);
        float segmentTime = distance / vehicleSpeed;
        
        // Add waiting time if segment is occupied
        if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
            segmentTime += estimateWaitTime(segmentId, vehicleId);
        }
        
        totalTime += segmentTime;
    }
    
    return totalTime;
}

float SegmentManager::estimateWaitTime(int segmentId, int vehicleId) const {
    const PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment || !segment->isOccupied) return 0.0f;
    
    // Base wait time estimate
    float baseWaitTime = 3.0f; // 3 seconds base wait
    
    // Add time based on queue position
    auto it = std::find(segment->queuedVehicles.begin(), segment->queuedVehicles.end(), vehicleId);
    if (it != segment->queuedVehicles.end()) {
        int queuePosition = std::distance(segment->queuedVehicles.begin(), it);
        baseWaitTime += queuePosition * 2.0f; // 2 seconds per vehicle ahead in queue
    }
    
    return baseWaitTime;
}

bool SegmentManager::shouldWaitOrReroute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const {
    // Calculate wait time for blocked segment
    float waitTime = estimateWaitTime(blockedSegmentId, vehicleId);
    
    // Find alternative path
    std::vector<int> altPath = findAvailablePath(currentNodeId, targetNodeId, vehicleId);
    
    if (altPath.empty()) {
        // No alternative, must wait
        return true;
    }
    
    // Calculate time for alternative path
    float altTime = estimatePathTime(altPath, vehicleId);
    
    // Add current segment time to original path
    const PathSegment* originalSegment = pathSystem->getSegment(blockedSegmentId);
    float originalTime = waitTime;
    if (originalSegment) {
        const PathNode* startNode = pathSystem->getNode(originalSegment->startNodeId);
        const PathNode* endNode = pathSystem->getNode(originalSegment->endNodeId);
        if (startNode && endNode) {
            float distance = startNode->position.distanceTo(endNode->position);
            originalTime += distance / 100.0f; // Add travel time
        }
    }
    
    std::cout << "Vehicle " << vehicleId << " decision: Wait time " << waitTime 
              << "s vs Alternative time " << altTime << "s" << std::endl;
    
    // Choose waiting if it's significantly faster (at least 1 second difference)
    return (originalTime + 1.0f) < altTime;
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
