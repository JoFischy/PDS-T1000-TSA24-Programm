#pragma once
#include "path_system.h"
#include <unordered_map>
#include <vector>
#include <queue> // Added for std::queue

class SegmentManager {
public:
    SegmentManager(PathSystem* pathSys);

    // Segment reservation system
    bool canVehicleEnterSegment(int segmentId, int vehicleId) const;
    bool reserveSegment(int segmentId, int vehicleId);
    void releaseSegment(int segmentId, int vehicleId);

    // Queue management
    void addToQueue(int segmentId, int vehicleId);
    void removeFromQueue(int segmentId, int vehicleId);
    void processQueue(int segmentId);
    void updateQueues();

    // Vehicle management
    int getVehicleSegment(int vehicleId) const;
    void removeVehicle(int vehicleId);

    // Path finding with traffic awareness
    std::vector<int> findAvailablePath(int startNodeId, int endNodeId, int vehicleId) const;
    std::vector<int> findOptimalPath(int startNodeId, int endNodeId, int vehicleId) const;
    bool isPathClear(const std::vector<int>& path, int vehicleId) const;

    // Time estimation methods
    float estimatePathTime(const std::vector<int>& path, int vehicleId) const;
    float estimateWaitTime(int segmentId, int vehicleId) const;

    bool shouldWaitOrReroute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;

    // Curve point detection
    bool isCurvePoint(int nodeId) const;
    std::vector<int> getCombinedCurveSegments(int nodeId) const;

    // T-junction evasion logic
    bool isTJunction(int nodeId) const;
    bool canUseEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;
    std::vector<int> findEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;
    bool isDeadlockSituation(int nodeId, int vehicleId, int otherVehicleId) const;

    // Status and debugging
    std::vector<int> getOccupiedSegments() const;
    void printSegmentStatus() const;

private:
    PathSystem* pathSystem;
    std::unordered_map<int, std::queue<int>> segmentQueues; // segmentId -> queue of vehicleIds
    std::unordered_map<int, int> vehicleToSegment; // vehicleId -> segmentId
    std::unordered_map<int, float> segmentReserveTime; // segmentId -> time of reservation expiry
    std::unordered_map<int, std::vector<int>> reservedCurveSegments; // vehicleId -> list of reserved segments for curves
};