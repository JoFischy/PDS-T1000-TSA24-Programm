#pragma once
#include "path_system.h"
#include <vector>
#include <unordered_map>

class SegmentManager {
public:
    SegmentManager(PathSystem* pathSys);

    // Segment reservation
    bool reserveSegment(int segmentId, int vehicleId);
    void releaseSegment(int segmentId, int vehicleId);
    bool isSegmentOccupied(int segmentId) const;

    // Queue management
    void addToQueue(int segmentId, int vehicleId);
    void removeFromQueue(int segmentId, int vehicleId);
    bool isVehicleInQueue(int segmentId, int vehicleId) const;
    int getNextInQueue(int segmentId) const;
    void updateQueues();
    int getSegmentOccupant(int segmentId) const;
    std::vector<int> getVehiclesReadyToMove() const;
    size_t getQueueLength(int segmentId) const;


    // Vehicle tracking
    void setVehicleSegment(int vehicleId, int segmentId);
    int getVehicleSegment(int vehicleId) const;
    void removeVehicle(int vehicleId);

    // Advanced path finding with traffic considerations
    std::vector<int> findAvailablePath(int startNodeId, int endNodeId, int vehicleId) const;
    std::vector<int> findOptimalPath(int startNodeId, int endNodeId, int vehicleId) const;
    bool canVehicleEnterSegment(int segmentId, int vehicleId) const;
    bool isPathClear(const std::vector<int>& path, int vehicleId) const;

    // Traffic analysis
    int getSegmentCongestion(int segmentId) const;
    std::vector<int> getAlternativePaths(int startNodeId, int endNodeId, int vehicleId) const;

private:
    PathSystem* pathSystem;
    std::unordered_map<int, int> vehicleSegmentMap; // vehicleId -> segmentId
};