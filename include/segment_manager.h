#pragma once
#include "path_system.h"
#include <unordered_map>
#include <vector>
#include <queue> // Added for std::queue
#include <set>   // REQUIRED for std::set - DO NOT REMOVE!

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

    // Node type detection
    enum class NodeType { JUNCTION, WAITING, CURVE, REGULAR };
    NodeType getNodeType(int nodeId) const;
    bool isJunctionNode(int nodeId) const;
    bool isWaitingNode(int nodeId) const;
    bool isCurveNode(int nodeId) const;
    
    // Advanced conflict detection and resolution
    struct ConflictInfo {
        int vehicleId;
        int targetJunctionId;
        int approachSegmentId;
        int exitSegmentId;
        float arrivalTime;
    };
    
    std::vector<ConflictInfo> detectPotentialConflicts(int vehicleId, const std::vector<int>& plannedPath) const;
    bool shouldWaitAtWaitingNode(int vehicleId, const std::vector<ConflictInfo>& conflicts) const;
    
    // Additional helper methods for conflict detection
    std::vector<int> findVehiclesApproachingJunction(int junctionId, int excludeVehicleId, float timeWindow) const;
    bool isJunctionCurrentlyOccupied(int junctionId, int excludeVehicleId) const;
    bool hasOpposingTraffic(int junctionId, int vehicleId) const;
    
    bool negotiatePassage(int vehicleId, int junctionId, const std::vector<int>& conflictingVehicles) const;
    
    // T-junction evasion logic (updated)
    bool isTJunction(int nodeId) const;
    bool canUseEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;
    std::vector<int> findEvasionRoute(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;
    
    // New T-junction conflict resolution methods
    bool handleTJunctionConflict(int currentNodeId, int targetNodeId, int blockedSegmentId, int vehicleId) const;
    int findConflictingVehicle(int currentNodeId, int vehicleId) const;
    bool vehiclesWantOppositeDirections(int currentNodeId, int vehicleId1, int vehicleId2) const;
    bool shouldUseEvasionSegment(int currentNodeId, int vehicleId, int conflictingVehicleId) const;
    int findEvasionSegment(int currentNodeId, int blockedSegmentId) const;
    int getVehicleWaitingNode(int currentNodeId, int vehicleId) const;
    bool isDeadlockSituation(int nodeId, int vehicleId, int otherVehicleId) const;
    bool hasConflictingTJunctionReservation(int tJunctionId, int vehicleId, int requestedSegmentId) const;

    // Deadlock detection and resolution
    bool detectDeadlock(const std::vector<int>& segmentsToReserve, int vehicleId) const;
    bool detectCircularWait(int startVehicleId, int currentVehicleId, std::set<int>& checkedVehicles) const;
    bool resolveDeadlock(int vehicleId, int blockedSegmentId);
    std::set<int> findDeadlockCycle(int startVehicleId) const;
    bool findCycleRecursive(int startVehicleId, int currentVehicleId, std::set<int>& visited, 
                           std::vector<int>& path, std::set<int>& cycleVehicles) const;
    void clearDeadlockQueues(const std::set<int>& deadlockVehicles);
    bool isVehicleWaitingForOurSegments(int waitingVehicleId, int ourVehicleId) const;
    bool segmentsConflict(int seg1, int seg2, int vehicle1, int vehicle2) const;
    
    // Consolidated segment handling (junction to junction)
    std::vector<int> getConsolidatedSegmentGroup(int segmentId) const;
    void checkAndAddConnectedSegments(int nodeId, std::vector<int>& toProcess, std::set<int>& processed) const;
    
    // Check if vehicle needs rerouting due to deadlock
    bool isVehicleDeadlocked(int vehicleId) const { return deadlockedVehicles.find(vehicleId) != deadlockedVehicles.end(); }
    void clearDeadlockFlag(int vehicleId) { deadlockedVehicles.erase(vehicleId); }

    // Status and debugging
    std::vector<int> getOccupiedSegments() const;
    void printSegmentStatus() const;

private:
    PathSystem* pathSystem;
    std::unordered_map<int, std::queue<int>> segmentQueues; // segmentId -> queue of vehicleIds
    std::unordered_map<int, int> vehicleToSegment; // vehicleId -> segmentId
    std::unordered_map<int, float> segmentReserveTime; // segmentId -> time of reservation expiry
    std::unordered_map<int, std::vector<int>> reservedCurveSegments; // vehicleId -> list of reserved segments for curves
    std::set<int> deadlockedVehicles; // Track vehicles that need rerouting due to deadlock
};