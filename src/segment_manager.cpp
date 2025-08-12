
#include "segment_manager.h"
#include <algorithm>
#include <iostream>
#include <set>

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
    
    // Get the consolidated segment group that includes waiting nodes
    std::vector<int> segmentsToReserve = getConsolidatedSegmentGroup(segmentId);
    
    // Check for deadlock situation first
    if (detectDeadlock(segmentsToReserve, vehicleId)) {
        std::cout << "Deadlock detected for vehicle " << vehicleId << " on segment " << segmentId << std::endl;
        if (resolveDeadlock(vehicleId, segmentId)) {
            std::cout << "Deadlock resolved for vehicle " << vehicleId << std::endl;
            return false; // Vehicle should replan route
        }
        return false;
    }
    
    // Check if all segments can be reserved
    for (int segId : segmentsToReserve) {
        PathSegment* seg = pathSystem->getSegment(segId);
        if (!seg) return false;
        
        // If occupied by another vehicle, cannot reserve
        if (seg->isOccupied && seg->occupiedByVehicleId != vehicleId) {
            std::cout << "Segment " << segId << " is occupied by vehicle " 
                      << seg->occupiedByVehicleId << ", vehicle " << vehicleId << " must wait" << std::endl;
            return false;
        }
    }
    
    // Reserve all segments in the group
    bool success = true;
    for (int segId : segmentsToReserve) {
        PathSegment* seg = pathSystem->getSegment(segId);
        if (seg && (!seg->isOccupied || seg->occupiedByVehicleId == vehicleId)) {
            seg->isOccupied = true;
            seg->occupiedByVehicleId = vehicleId;
            vehicleToSegment[vehicleId] = segmentId; // Store main segment
        } else {
            success = false;
            break;
        }
    }
    
    if (success) {
        std::cout << "Vehicle " << vehicleId << " reserved consolidated segment group starting with " << segmentId << std::endl;
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
    // New T-junction conflict resolution logic
    if (isTJunction(currentNodeId)) {
        return handleTJunctionConflict(currentNodeId, targetNodeId, blockedSegmentId, vehicleId);
    }
    
    // Original logic for non-T-junctions
    float waitTime = estimateWaitTime(blockedSegmentId, vehicleId);
    std::vector<int> altPath = findAvailablePath(currentNodeId, targetNodeId, vehicleId);
    
    if (altPath.empty()) {
        return true;
    }
    
    float altTime = estimatePathTime(altPath, vehicleId);
    const PathSegment* originalSegment = pathSystem->getSegment(blockedSegmentId);
    float originalTime = waitTime;
    if (originalSegment) {
        const PathNode* startNode = pathSystem->getNode(originalSegment->startNodeId);
        const PathNode* endNode = pathSystem->getNode(originalSegment->endNodeId);
        if (startNode && endNode) {
            float distance = startNode->position.distanceTo(endNode->position);
            originalTime += distance / 100.0f;
        }
    }
    
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



SegmentManager::NodeType SegmentManager::getNodeType(int nodeId) const {
    const PathNode* node = pathSystem->getNode(nodeId);
    if (!node) return NodeType::REGULAR;
    
    if (node->isWaitingNode) return NodeType::WAITING;
    
    int connectionCount = node->connectedSegments.size();
    
    if (connectionCount >= 3) {
        return NodeType::JUNCTION;
    } else if (connectionCount == 2) {
        return NodeType::CURVE;
    }
    
    return NodeType::REGULAR;
}

bool SegmentManager::isJunctionNode(int nodeId) const {
    return getNodeType(nodeId) == NodeType::JUNCTION;
}

bool SegmentManager::isWaitingNode(int nodeId) const {
    return getNodeType(nodeId) == NodeType::WAITING;
}

bool SegmentManager::isCurveNode(int nodeId) const {
    return getNodeType(nodeId) == NodeType::CURVE;
}

bool SegmentManager::isTJunction(int nodeId) const {
    const PathNode* node = pathSystem->getNode(nodeId);
    if (!node) return false;
    
    // T-junction has exactly 3 connected segments
    return node->connectedSegments.size() == 3;
}

std::vector<SegmentManager::ConflictInfo> SegmentManager::detectPotentialConflicts(int vehicleId, const std::vector<int>& plannedPath) const {
    std::vector<ConflictInfo> conflicts;
    
    if (plannedPath.empty()) return conflicts;
    
    // Analyze each junction in the planned path
    float currentTime = 0.0f;
    const float vehicleSpeed = 100.0f;
    
    for (size_t i = 0; i < plannedPath.size(); i++) {
        int segmentId = plannedPath[i];
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        // Add travel time for this segment
        currentTime += segment->length / vehicleSpeed;
        
        // Check if this segment leads to a junction
        int junctionId = -1;
        if (isJunctionNode(segment->startNodeId)) {
            junctionId = segment->startNodeId;
        } else if (isJunctionNode(segment->endNodeId)) {
            junctionId = segment->endNodeId;
        }
        
        if (junctionId != -1) {
            // Find other vehicles that might conflict at this junction
            std::vector<int> conflictingVehicles = findVehiclesApproachingJunction(junctionId, vehicleId, currentTime);
            
            if (!conflictingVehicles.empty()) {
                ConflictInfo conflict;
                conflict.vehicleId = vehicleId;
                conflict.targetJunctionId = junctionId;
                conflict.approachSegmentId = segmentId;
                conflict.exitSegmentId = (i + 1 < plannedPath.size()) ? plannedPath[i + 1] : -1;
                conflict.arrivalTime = currentTime;
                conflicts.push_back(conflict);
            }
        }
    }
    
    return conflicts;
}

std::vector<int> SegmentManager::findVehiclesApproachingJunction(int junctionId, int excludeVehicleId, float timeWindow) const {
    std::vector<int> conflictingVehicles;
    
    const PathNode* junction = pathSystem->getNode(junctionId);
    if (!junction) return conflictingVehicles;
    
    // Check all segments connected to this junction
    for (int segmentId : junction->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        // Check if occupied by another vehicle
        if (segment->isOccupied && segment->occupiedByVehicleId != excludeVehicleId) {
            conflictingVehicles.push_back(segment->occupiedByVehicleId);
        }
        
        // Check queued vehicles
        for (int queuedVehicle : segment->queuedVehicles) {
            if (queuedVehicle != excludeVehicleId) {
                conflictingVehicles.push_back(queuedVehicle);
            }
        }
    }
    
    return conflictingVehicles;
}

bool SegmentManager::shouldWaitAtWaitingNode(int vehicleId, const std::vector<ConflictInfo>& conflicts) const {
    if (conflicts.empty()) return false;
    
    // If there are multiple conflicts or high-priority conflicts, wait
    for (const auto& conflict : conflicts) {
        // Check if junction is currently occupied
        if (isJunctionCurrentlyOccupied(conflict.targetJunctionId, vehicleId)) {
            return true;
        }
        
        // Check if there are opposing traffic flows
        if (hasOpposingTraffic(conflict.targetJunctionId, vehicleId)) {
            return true;
        }
    }
    
    return false;
}

bool SegmentManager::isJunctionCurrentlyOccupied(int junctionId, int excludeVehicleId) const {
    const PathNode* junction = pathSystem->getNode(junctionId);
    if (!junction) return false;
    
    // Check if any connected segment is occupied
    for (int segmentId : junction->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (segment && segment->isOccupied && segment->occupiedByVehicleId != excludeVehicleId) {
            return true;
        }
    }
    
    return false;
}

bool SegmentManager::hasOpposingTraffic(int junctionId, int vehicleId) const {
    // This would need vehicle controller integration to check vehicle intentions
    // For now, return false as placeholder
    return false;
}

bool SegmentManager::negotiatePassage(int vehicleId, int junctionId, const std::vector<int>& conflictingVehicles) const {
    if (conflictingVehicles.empty()) return true;
    
    // Simple priority system: lower vehicle ID has priority
    for (int otherVehicleId : conflictingVehicles) {
        if (otherVehicleId < vehicleId) {
            return false; // Other vehicle has priority
        }
    }
    
    return true; // This vehicle has priority
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
    int evasionSegmentId = findEvasionSegment(currentNodeId, blockedSegmentId);
    if (evasionSegmentId == -1) return {};
    
    // Check if evasion segment is actually free
    const PathSegment* evasionSegment = pathSystem->getSegment(evasionSegmentId);
    if (!evasionSegment || (evasionSegment->isOccupied && evasionSegment->occupiedByVehicleId != vehicleId)) {
        return {}; // Evasion route blocked
    }
    
    int evasionNodeId = (evasionSegment->startNodeId == currentNodeId) ? 
                        evasionSegment->endNodeId : evasionSegment->startNodeId;
    
    // Build evasion path: go to evasion node and wait there
    std::vector<int> evasionPath;
    evasionPath.push_back(evasionSegmentId);
    
    // After waiting at evasion node, find path back to target
    std::vector<int> returnPath = pathSystem->findPath(evasionNodeId, targetNodeId, {});
    evasionPath.insert(evasionPath.end(), returnPath.begin(), returnPath.end());
    
    return evasionPath;
}

bool SegmentManager::handleTJunctionConflict(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const {
    int conflictingVehicleId = findConflictingVehicle(currentNodeId, vehicleId);
    if (conflictingVehicleId == -1) {
        // No conflicting vehicle, just wait normally
        return true;
    }
    
    // Case 1: Both vehicles want opposite directions
    if (vehiclesWantOppositeDirections(currentNodeId, vehicleId, conflictingVehicleId)) {
        // First vehicle should use evasion segment
        if (shouldUseEvasionSegment(currentNodeId, vehicleId, conflictingVehicleId)) {
            return false; // Use evasion route
        } else {
            return true; // Wait at waiting node
        }
    }
    
    // Case 2: Only one wants opposite direction - free route goes first
    // Case 3: Both want same direction - first arrival goes first
    // In both cases, the vehicle with blocked route should wait
    return true;
}

int SegmentManager::findConflictingVehicle(int currentNodeId, int vehicleId) const {
    const PathNode* node = pathSystem->getNode(currentNodeId);
    if (!node) return -1;
    
    // Check all segments connected to this T-junction for other vehicles
    for (int segmentId : node->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (segment && segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
            // Found another vehicle approaching this junction
            return segment->occupiedByVehicleId;
        }
        
        // Also check queued vehicles
        for (int queuedVehicleId : segment->queuedVehicles) {
            if (queuedVehicleId != vehicleId) {
                return queuedVehicleId;
            }
        }
    }
    
    return -1;
}

bool SegmentManager::vehiclesWantOppositeDirections(int currentNodeId, int vehicleId1, int vehicleId2) const {
    // Get target directions for both vehicles
    // This would need access to vehicle controller to get target nodes
    // For now, return false as a placeholder
    return false;
}

bool SegmentManager::shouldUseEvasionSegment(int currentNodeId, int vehicleId, int conflictingVehicleId) const {
    // Simple logic: first vehicle by ID uses evasion segment
    return vehicleId < conflictingVehicleId;
}

int SegmentManager::findEvasionSegment(int currentNodeId, int blockedSegmentId) const {
    const PathNode* node = pathSystem->getNode(currentNodeId);
    if (!node || !isTJunction(currentNodeId)) return -1;
    
    // Find a free segment that's not the blocked one
    std::vector<int> availableSegments;
    
    for (int segmentId : node->connectedSegments) {
        if (segmentId != blockedSegmentId) {
            const PathSegment* segment = pathSystem->getSegment(segmentId);
            if (segment && !segment->isOccupied) {
                availableSegments.push_back(segmentId);
            }
        }
    }
    
    // Return first available segment
    return availableSegments.empty() ? -1 : availableSegments[0];
}

int SegmentManager::getVehicleWaitingNode(int currentNodeId, int vehicleId) const {
    const PathNode* node = pathSystem->getNode(currentNodeId);
    if (!node) return -1;
    
    // Find waiting node connected to this T-junction
    for (int segmentId : node->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        // Check if other end is a waiting node
        int otherNodeId = (segment->startNodeId == currentNodeId) ? 
                         segment->endNodeId : segment->startNodeId;
        const PathNode* otherNode = pathSystem->getNode(otherNodeId);
        if (otherNode && otherNode->isWaitingNode) {
            return otherNodeId;
        }
    }
    
    return -1;
}

bool SegmentManager::hasConflictingTJunctionReservation(int tJunctionId, int vehicleId, int requestedSegmentId) const {
    const PathNode* tJunction = pathSystem->getNode(tJunctionId);
    if (!tJunction) return false;
    
    // Count how many segments connected to this T-junction are occupied
    int occupiedSegments = 0;
    int conflictingVehicleId = -1;
    
    for (int segmentId : tJunction->connectedSegments) {
        const PathSegment* segment = pathSystem->getSegment(segmentId);
        if (!segment) continue;
        
        if (segment->isOccupied && segment->occupiedByVehicleId != vehicleId) {
            occupiedSegments++;
            conflictingVehicleId = segment->occupiedByVehicleId;
        }
    }
    
    // If there's already another vehicle approaching this T-junction, create conflict
    if (occupiedSegments > 0) {
        std::cout << "T-junction " << tJunctionId << " has conflict: vehicle " << vehicleId 
                  << " vs vehicle " << conflictingVehicleId << std::endl;
        return true;
    }
    
    return false;
}

std::vector<int> SegmentManager::getConsolidatedSegmentGroup(int segmentId) const {
    std::vector<int> segmentGroup;
    std::vector<int> toProcess = {segmentId};
    std::set<int> processed;
    
    while (!toProcess.empty()) {
        int currentSegId = toProcess.back();
        toProcess.pop_back();
        
        if (processed.find(currentSegId) != processed.end()) continue;
        processed.insert(currentSegId);
        
        const PathSegment* segment = pathSystem->getSegment(currentSegId);
        if (!segment) continue;
        
        segmentGroup.push_back(currentSegId);
        
        // Check both endpoints
        checkAndAddConnectedSegments(segment->startNodeId, toProcess, processed);
        checkAndAddConnectedSegments(segment->endNodeId, toProcess, processed);
    }
    
    return segmentGroup;
}

void SegmentManager::checkAndAddConnectedSegments(int nodeId, std::vector<int>& toProcess, std::set<int>& processed) const {
    const PathNode* node = pathSystem->getNode(nodeId);
    if (!node) return;
    
    // If this is a waiting node or curve point, add connected segments
    if (node->isWaitingNode || isCurvePoint(nodeId)) {
        for (int connectedSegId : node->connectedSegments) {
            if (processed.find(connectedSegId) == processed.end()) {
                toProcess.push_back(connectedSegId);
            }
        }
    }
}

bool SegmentManager::detectDeadlock(const std::vector<int>& segmentsToReserve, int vehicleId) const {
    // Enhanced deadlock detection for circular waiting
    std::set<int> checkedVehicles;
    return detectCircularWait(vehicleId, vehicleId, checkedVehicles);
}

bool SegmentManager::detectCircularWait(int startVehicleId, int currentVehicleId, std::set<int>& checkedVehicles) const {
    if (checkedVehicles.find(currentVehicleId) != checkedVehicles.end()) {
        // Found cycle - this is a deadlock
        return currentVehicleId == startVehicleId;
    }
    
    checkedVehicles.insert(currentVehicleId);
    
    // Find what segments this vehicle is waiting for
    for (const auto& segment : pathSystem->getSegments()) {
        if (!segment.queuedVehicles.empty()) {
            auto it = std::find(segment.queuedVehicles.begin(), segment.queuedVehicles.end(), currentVehicleId);
            if (it != segment.queuedVehicles.end()) {
                // This vehicle is waiting for this segment
                if (segment.isOccupied && segment.occupiedByVehicleId != currentVehicleId) {
                    // Check if the occupying vehicle creates a cycle
                    if (detectCircularWait(startVehicleId, segment.occupiedByVehicleId, checkedVehicles)) {
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}

bool SegmentManager::isVehicleWaitingForOurSegments(int waitingVehicleId, int ourVehicleId) const {
    // Check all segments to see if the waiting vehicle is queued for any segment we occupy
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied && segment.occupiedByVehicleId == ourVehicleId) {
            // Check if waiting vehicle is in queue for this segment
            for (int queuedVehicle : segment.queuedVehicles) {
                if (queuedVehicle == waitingVehicleId) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool SegmentManager::resolveDeadlock(int vehicleId, int blockedSegmentId) {
    std::cout << "Attempting to resolve deadlock for vehicle " << vehicleId << std::endl;
    
    // Find all vehicles involved in the deadlock cycle
    std::set<int> deadlockVehicles = findDeadlockCycle(vehicleId);
    
    if (deadlockVehicles.empty()) {
        deadlockVehicles.insert(vehicleId); // At least include the requesting vehicle
    }
    
    // Choose vehicle to reroute (prefer vehicle with highest ID to be deterministic)
    int vehicleToReroute = *std::max_element(deadlockVehicles.begin(), deadlockVehicles.end());
    
    std::cout << "Resolving deadlock by rerouting vehicle " << vehicleToReroute << std::endl;
    
    // Release all segments held by the chosen vehicle
    int currentSegment = getVehicleSegment(vehicleToReroute);
    if (currentSegment != -1) {
        releaseSegment(currentSegment, vehicleToReroute);
        std::cout << "Released current segment " << currentSegment << " for vehicle " << vehicleToReroute << std::endl;
    }
    
    // Remove chosen vehicle from all queues
    for (const auto& segment : pathSystem->getSegments()) {
        removeFromQueue(segment.segmentId, vehicleToReroute);
    }
    
    // Mark chosen vehicle for rerouting
    deadlockedVehicles.insert(vehicleToReroute);
    
    // Also clear queues for segments that were causing the deadlock
    clearDeadlockQueues(deadlockVehicles);
    
    return true;
}

std::set<int> SegmentManager::findDeadlockCycle(int startVehicleId) const {
    std::set<int> cycleVehicles;
    std::set<int> visitedVehicles;
    std::vector<int> currentPath;
    
    if (findCycleRecursive(startVehicleId, startVehicleId, visitedVehicles, currentPath, cycleVehicles)) {
        return cycleVehicles;
    }
    
    return std::set<int>(); // No cycle found
}

bool SegmentManager::findCycleRecursive(int startVehicleId, int currentVehicleId, 
                                      std::set<int>& visited, std::vector<int>& path, 
                                      std::set<int>& cycleVehicles) const {
    if (std::find(path.begin(), path.end(), currentVehicleId) != path.end()) {
        // Found cycle, add all vehicles in the cycle
        bool inCycle = false;
        for (int vehicleId : path) {
            if (vehicleId == currentVehicleId) {
                inCycle = true;
            }
            if (inCycle) {
                cycleVehicles.insert(vehicleId);
            }
        }
        return true;
    }
    
    if (visited.find(currentVehicleId) != visited.end()) {
        return false; // Already explored this path
    }
    
    visited.insert(currentVehicleId);
    path.push_back(currentVehicleId);
    
    // Find what vehicle this vehicle is waiting for
    for (const auto& segment : pathSystem->getSegments()) {
        auto it = std::find(segment.queuedVehicles.begin(), segment.queuedVehicles.end(), currentVehicleId);
        if (it != segment.queuedVehicles.end() && segment.isOccupied) {
            int blockedByVehicle = segment.occupiedByVehicleId;
            if (findCycleRecursive(startVehicleId, blockedByVehicle, visited, path, cycleVehicles)) {
                return true;
            }
        }
    }
    
    path.pop_back();
    return false;
}

void SegmentManager::clearDeadlockQueues(const std::set<int>& deadlockVehicles) {
    // Get non-const access to segments for modification
    for (int i = 0; i < pathSystem->getSegmentCount(); ++i) {
        PathSegment* segment = pathSystem->getSegment(i);
        if (segment) {
            auto& queue = segment->queuedVehicles;
            queue.erase(std::remove_if(queue.begin(), queue.end(),
                [&deadlockVehicles](int vehicleId) {
                    return deadlockVehicles.find(vehicleId) != deadlockVehicles.end();
                }), queue.end());
        }
    }
}

bool SegmentManager::isDeadlockSituation(int nodeId, int vehicleId, int otherVehicleId) const {
    // Enhanced deadlock detection considering consolidated segments
    
    // Get consolidated segment groups for both vehicles
    int vehicleSegment = getVehicleSegment(vehicleId);
    int otherSegment = getVehicleSegment(otherVehicleId);
    
    if (vehicleSegment == -1 || otherSegment == -1) return false;
    
    std::vector<int> vehicleGroup = getConsolidatedSegmentGroup(vehicleSegment);
    std::vector<int> otherGroup = getConsolidatedSegmentGroup(otherSegment);
    
    // Check if there's a circular dependency in segment reservations
    for (int vSeg : vehicleGroup) {
        for (int oSeg : otherGroup) {
            const PathSegment* vSegment = pathSystem->getSegment(vSeg);
            const PathSegment* oSegment = pathSystem->getSegment(oSeg);
            
            if (!vSegment || !oSegment) continue;
            
            // Check for conflicting reservations that could cause deadlock
            if (segmentsConflict(vSeg, oSeg, vehicleId, otherVehicleId)) {
                return true;
            }
        }
    }
    
    return false;
}

bool SegmentManager::segmentsConflict(int seg1, int seg2, int vehicle1, int vehicle2) const {
    const PathSegment* segment1 = pathSystem->getSegment(seg1);
    const PathSegment* segment2 = pathSystem->getSegment(seg2);
    
    if (!segment1 || !segment2) return false;
    
    // Check if vehicles want to move in opposite directions on overlapping paths
    bool sharesNode = (segment1->startNodeId == segment2->startNodeId ||
                       segment1->startNodeId == segment2->endNodeId ||
                       segment1->endNodeId == segment2->startNodeId ||
                       segment1->endNodeId == segment2->endNodeId);
    
    if (sharesNode) {
        // Check if one is occupied by the other vehicle and vice versa
        bool seg1BlocksSeg2 = (segment1->isOccupied && segment1->occupiedByVehicleId == vehicle1);
        bool seg2BlocksSeg1 = (segment2->isOccupied && segment2->occupiedByVehicleId == vehicle2);
        
        return seg1BlocksSeg2 && seg2BlocksSeg1;
    }
    
    return false;
}
