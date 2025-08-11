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
    // Zuerst optimale Route ohne Verkehrsberücksichtigung versuchen
    std::vector<int> optimalPath = findOptimalPath(startNodeId, endNodeId, vehicleId);

    if (isPathClear(optimalPath, vehicleId)) {
        return optimalPath;
    }

    // Wenn optimale Route blockiert ist, alternative Routen suchen
    std::vector<int> alternatives = getAlternativePaths(startNodeId, endNodeId, vehicleId);

    for (const auto& path : {optimalPath}) {
        if (isPathClear(path, vehicleId)) {
            return path;
        }
    }

    // Wenn keine freie Route verfügbar, beste verfügbare Route mit minimaler Blockierung
    std::vector<int> excludedSegments;
    for (const auto& segment : pathSystem->getSegments()) {
        if (segment.isOccupied && segment.occupiedByVehicleId != vehicleId) {
            // Nur Segmente mit hoher Staukongestion ausschließen
            if (getSegmentCongestion(segment.segmentId) > 2) {
                excludedSegments.push_back(segment.segmentId);
            }
        }
    }

    return pathSystem->findPath(startNodeId, endNodeId, excludedSegments);
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