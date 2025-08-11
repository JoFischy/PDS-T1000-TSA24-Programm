#include "segment_manager.h"
#include <algorithm>

SegmentManager::SegmentManager(PathSystem* pathSys) : pathSystem(pathSys) {}

bool SegmentManager::reserveSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;
    
    if (segment->isOccupied) {
        // Add to queue if not already there
        if (!isVehicleInQueue(segmentId, vehicleId)) {
            addToQueue(segmentId, vehicleId);
        }
        return false;
    }
    
    // Reserve the segment
    segment->isOccupied = true;
    segment->occupiedByVehicleId = vehicleId;
    setVehicleSegment(vehicleId, segmentId);
    
    return true;
}

void SegmentManager::releaseSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment || segment->occupiedByVehicleId != vehicleId) return;
    
    // Release the segment
    segment->isOccupied = false;
    segment->occupiedByVehicleId = -1;
    
    // Remove vehicle from tracking
    vehicleSegmentMap.erase(vehicleId);
    
    // Process queue for this segment
    if (!segment->queuedVehicles.empty()) {
        // The next vehicle in queue will be processed in updateQueues()
    }
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
    // Get occupied segments to exclude from pathfinding
    std::vector<int> excludedSegments;
    
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied && segment.occupiedByVehicleId != vehicleId) {
            excludedSegments.push_back(segment.segmentId);
        }
    }
    
    return pathSystem->findPath(startNodeId, endNodeId, excludedSegments);
}

bool SegmentManager::canVehicleEnterSegment(int segmentId, int vehicleId) const {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return false;
    
    if (!segment->isOccupied) return true;
    
    // Can enter if already occupying this segment
    return segment->occupiedByVehicleId == vehicleId;
}

void SegmentManager::updateQueues() {
    // Process all segments and try to move queued vehicles
    for (const auto& segment : pathSystem->getSegments()) {
        if (!segment.isOccupied && !segment.queuedVehicles.empty()) {
            int nextVehicleId = segment.queuedVehicles.front();
            
            // Try to reserve the segment for the next vehicle in queue
            if (reserveSegment(segment.segmentId, nextVehicleId)) {
                // Remove from queue since it now occupies the segment
                PathSegment* seg = pathSystem->getSegment(segment.segmentId);
                if (seg) {
                    seg->queuedVehicles.erase(seg->queuedVehicles.begin());
                }
            }
        }
    }
}

std::vector<int> SegmentManager::getVehiclesReadyToMove() const {
    std::vector<int> readyVehicles;
    
    // Find vehicles that just got their reserved segments
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied) {
            readyVehicles.push_back(segment.occupiedByVehicleId);
        }
    }
    
    return readyVehicles;
}