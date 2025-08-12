
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
    
    // Check if this segment involves curve points
    std::vector<int> segmentsToReserve;
    segmentsToReserve.push_back(segmentId);
    
    // Check if start node is a curve point
    if (isCurvePoint(segment->startNodeId)) {
        auto curveSegments = getCombinedCurveSegments(segment->startNodeId);
        for (int curveSegId : curveSegments) {
            if (curveSegId != segmentId && 
                std::find(segmentsToReserve.begin(), segmentsToReserve.end(), curveSegId) == segmentsToReserve.end()) {
                segmentsToReserve.push_back(curveSegId);
            }
        }
    }
    
    // Check if end node is a curve point
    if (isCurvePoint(segment->endNodeId)) {
        auto curveSegments = getCombinedCurveSegments(segment->endNodeId);
        for (int curveSegId : curveSegments) {
            if (curveSegId != segmentId && 
                std::find(segmentsToReserve.begin(), segmentsToReserve.end(), curveSegId) == segmentsToReserve.end()) {
                segmentsToReserve.push_back(curveSegId);
            }
        }
    }
    
    // Check if all segments can be reserved
    for (int segId : segmentsToReserve) {
        PathSegment* seg = pathSystem->getSegment(segId);
        if (!seg) return false;
        
        // If occupied by another vehicle, cannot reserve
        if (seg->isOccupied && seg->occupiedByVehicleId != vehicleId) {
            std::cout << "Segment " << segId << " (part of curve) is occupied by vehicle " 
                      << seg->occupiedByVehicleId << ", vehicle " << vehicleId << " must wait" << std::endl;
            return false;
        }
    }
    
    // Reserve all segments
    bool success = true;
    for (int segId : segmentsToReserve) {
        PathSegment* seg = pathSystem->getSegment(segId);
        if (seg && (!seg->isOccupied || seg->occupiedByVehicleId == vehicleId)) {
            seg->isOccupied = true;
            seg->occupiedByVehicleId = vehicleId;
            vehicleToSegment[vehicleId] = segId; // Store main segment
            
            if (segId == segmentId) {
                std::cout << "Vehicle " << vehicleId << " reserved segment " << segmentId;
                if (segmentsToReserve.size() > 1) {
                    std::cout << " (including curve segments)";
                }
                std::cout << std::endl;
            }
        } else {
            success = false;
            break;
        }
    }
    
    return success;
}

void SegmentManager::releaseSegment(int segmentId, int vehicleId) {
    PathSegment* segment = pathSystem->getSegment(segmentId);
    if (!segment) return;
    
    // Only release if this vehicle actually occupies it
    if (segment->isOccupied && segment->occupiedByVehicleId == vehicleId) {
        // Find all segments that should be released together (curve handling)
        std::vector<int> segmentsToRelease;
        segmentsToRelease.push_back(segmentId);
        
        // Check if start node is a curve point
        if (isCurvePoint(segment->startNodeId)) {
            auto curveSegments = getCombinedCurveSegments(segment->startNodeId);
            for (int curveSegId : curveSegments) {
                PathSegment* curveSeg = pathSystem->getSegment(curveSegId);
                if (curveSeg && curveSeg->isOccupied && curveSeg->occupiedByVehicleId == vehicleId &&
                    curveSegId != segmentId) {
                    segmentsToRelease.push_back(curveSegId);
                }
            }
        }
        
        // Check if end node is a curve point
        if (isCurvePoint(segment->endNodeId)) {
            auto curveSegments = getCombinedCurveSegments(segment->endNodeId);
            for (int curveSegId : curveSegments) {
                PathSegment* curveSeg = pathSystem->getSegment(curveSegId);
                if (curveSeg && curveSeg->isOccupied && curveSeg->occupiedByVehicleId == vehicleId &&
                    curveSegId != segmentId &&
                    std::find(segmentsToRelease.begin(), segmentsToRelease.end(), curveSegId) == segmentsToRelease.end()) {
                    segmentsToRelease.push_back(curveSegId);
                }
            }
        }
        
        // Release all segments
        for (int segId : segmentsToRelease) {
            PathSegment* seg = pathSystem->getSegment(segId);
            if (seg && seg->isOccupied && seg->occupiedByVehicleId == vehicleId) {
                seg->isOccupied = false;
                seg->occupiedByVehicleId = -1;
                
                // Process any queued vehicles for this segment
                processQueue(segId);
            }
        }
        
        vehicleToSegment.erase(vehicleId);
        
        std::cout << "Vehicle " << vehicleId << " released segment " << segmentId;
        if (segmentsToRelease.size() > 1) {
            std::cout << " (including curve segments)";
        }
        std::cout << std::endl;
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
    auto it = vehicleToSegment.find(vehicleId);
    return (it != vehicleToSegment.end()) ? it->second : -1;
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

    vehicleToSegment.erase(vehicleId);
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
    // Simple decision: calculate wait time vs alternative path time
    float waitTime = estimateWaitTime(blockedSegmentId, vehicleId);
    
    // Find alternative path using waiting points
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

bool SegmentManager::isCurvePoint(int nodeId) const {
    const PathNode* node = pathSystem->getNode(nodeId);
    if (!node) return false;
    
    // Count connected segments
    int connectionCount = 0;
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.startNodeId == nodeId || segment.endNodeId == nodeId) {
            connectionCount++;
        }
    }
    
    // Node with exactly 2 connections is a curve point
    return connectionCount == 2;
}

std::vector<int> SegmentManager::getCombinedCurveSegments(int nodeId) const {
    std::vector<int> curveSegments;
    
    if (!isCurvePoint(nodeId)) {
        return curveSegments;
    }
    
    // Find the two segments connected to this curve point
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.startNodeId == nodeId || segment.endNodeId == nodeId) {
            curveSegments.push_back(segment.segmentId);
        }
    }
    
    return curveSegments;
}



bool SegmentManager::isTJunction(int nodeId) const {
    const PathNode* node = pathSystem->getNode(nodeId);
    if (!node) return false;
    
    // T-junction has exactly 3 connected segments
    return node->connectedSegments.size() == 3;
}

bool SegmentManager::canUseEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const {
    if (!isTJunction(currentNodeId)) return false;
    
    const PathNode* node = pathSystem->getNode(currentNodeId);
    if (!node) return false;
    
    // Find the third segment (evasion route)
    int evasionSegmentId = -1;
    for (int segmentId : node->connectedSegments) {
        if (segmentId != blockedSegmentId) {
            const PathSegment* segment = pathSystem->getSegment(segmentId);
            if (segment && !segment->isOccupied) {
                evasionSegmentId = segmentId;
                break;
            }
        }
    }
    
    if (evasionSegmentId == -1) return false;
    
    // Calculate if evasion route is time-efficient
    std::vector<int> evasionPath = findEvasionRoute(currentNodeId, targetNodeId, blockedSegmentId, vehicleId);
    if (evasionPath.empty()) return false;
    
    float evasionTime = estimatePathTime(evasionPath, vehicleId);
    float waitTime = estimateWaitTime(blockedSegmentId, vehicleId);
    
    // Use evasion if it's faster or only slightly slower (within 2 seconds)
    return evasionTime <= (waitTime + 2.0f);
}

std::vector<int> SegmentManager::findEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const {
    const PathNode* currentNode = pathSystem->getNode(currentNodeId);
    if (!currentNode || !isTJunction(currentNodeId)) return {};
    
    // Find the third segment for evasion
    int evasionSegmentId = -1;
    for (int segmentId : currentNode->connectedSegments) {
        if (segmentId != blockedSegmentId) {
            const PathSegment* segment = pathSystem->getSegment(segmentId);
            if (segment && !segment->isOccupied) {
                evasionSegmentId = segmentId;
                break;
            }
        }
    }
    
    if (evasionSegmentId == -1) return {};
    
    // Get the node at the end of evasion segment
    const PathSegment* evasionSegment = pathSystem->getSegment(evasionSegmentId);
    if (!evasionSegment) return {};
    
    int evasionNodeId = (evasionSegment->startNodeId == currentNodeId) ? 
                        evasionSegment->endNodeId : evasionSegment->startNodeId;
    
    // Build evasion path: current -> evasion node -> back to current -> target
    std::vector<int> evasionPath;
    
    // Step 1: Go to evasion node
    evasionPath.push_back(evasionSegmentId);
    
    // Step 2: Return to current node (reverse direction)
    evasionPath.push_back(evasionSegmentId);
    
    // Step 3: Continue to target via the originally blocked segment (should be free now)
    std::vector<int> remainingPath = pathSystem->findPath(currentNodeId, targetNodeId, {});
    evasionPath.insert(evasionPath.end(), remainingPath.begin(), remainingPath.end());
    
    return evasionPath;
}

bool SegmentManager::isDeadlockSituation(int nodeId, int vehicleId, int otherVehicleId) const {
    if (!isTJunction(nodeId)) return false;
    
    // Get the segments both vehicles want to use
    int vehicleSegment = getVehicleSegment(vehicleId);
    int otherSegment = getVehicleSegment(otherVehicleId);
    
    if (vehicleSegment == -1 || otherSegment == -1) return false;
    
    const PathSegment* vSeg = pathSystem->getSegment(vehicleSegment);
    const PathSegment* oSeg = pathSystem->getSegment(otherSegment);
    
    if (!vSeg || !oSeg) return false;
    
    // Check if both vehicles want to go to where the other is coming from
    bool deadlock = false;
    
    // Vehicle 1 wants to go to where Vehicle 2 is coming from
    if ((vSeg->startNodeId == nodeId && oSeg->endNodeId == nodeId) ||
        (vSeg->endNodeId == nodeId && oSeg->startNodeId == nodeId)) {
        deadlock = true;
    }
    
    return deadlock;
}
