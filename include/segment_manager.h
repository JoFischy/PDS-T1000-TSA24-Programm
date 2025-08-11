#pragma once
#include "path_system.h"
#include <vector>
#include <unordered_map>

class SegmentManager {
public:
    explicit SegmentManager(PathSystem* pathSystem);
    
    // Segment reservation and release
    bool reserveSegment(int segmentId, int vehicleId);
    void releaseSegment(int segmentId, int vehicleId);
    
    // Status queries
    bool isSegmentOccupied(int segmentId) const;
    int getSegmentOccupant(int segmentId) const;
    bool canVehicleEnterSegment(int segmentId, int vehicleId) const;
    
    // Queue management
    void addToQueue(int segmentId, int vehicleId);
    void removeFromQueue(int segmentId, int vehicleId);
    int getNextInQueue(int segmentId) const;
    bool isVehicleInQueue(int segmentId, int vehicleId) const;
    size_t getQueueLength(int segmentId) const;
    
    // Vehicle tracking
    void setVehicleSegment(int vehicleId, int segmentId);
    int getVehicleSegment(int vehicleId) const;
    void removeVehicle(int vehicleId);
    
    // Path planning with exclusions
    std::vector<int> findAvailablePath(int startNodeId, int endNodeId, int vehicleId) const;
    
    // Update system
    void updateQueues();
    std::vector<int> getVehiclesReadyToMove() const;
    
private:
    PathSystem* pathSystem;
    std::unordered_map<int, int> vehicleSegmentMap; // vehicleId -> segmentId
};