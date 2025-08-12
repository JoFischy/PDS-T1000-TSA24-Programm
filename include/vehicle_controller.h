#pragma once
#include "auto.h"
#include "path_system.h"
#include "segment_manager.h"
#include <vector>
#include <unordered_map>

// Enum definitions for vehicle behavior
struct VehicleIntention {
    enum Type {
        GO_STRAIGHT,
        TURN_LEFT,
        TURN_RIGHT,
        UNKNOWN
    };
    
    Type type;
    Direction targetDirection;
    
    VehicleIntention() : type(UNKNOWN), targetDirection(Direction::NORTH) {}
    VehicleIntention(Type t, Direction dir) : type(t), targetDirection(dir) {}
};

// Note: Direction enum is defined in movement_system.h (included via auto.h)

class VehicleController {
public:
    VehicleController(PathSystem* pathSys, SegmentManager* segmentMgr);

    // Vehicle management
    int addVehicle(const Point& startPosition);
    void removeVehicle(int vehicleId);
    void spawnInitialVehicles(); // Add 4 vehicles at different nodes
    void assignRandomTargetsToAllVehicles(); // Give all vehicles random targets
    Auto* getVehicle(int vehicleId);
    const Auto* getVehicle(int vehicleId) const;

    // Smart target assignment
    void setVehicleTargetNode(int vehicleId, int targetNodeId);
    void assignNewRandomTarget(int vehicleId);
    bool isVehicleAtTarget(int vehicleId) const;

    // Movement control
    void setVehicleTarget(int vehicleId, const Point& targetPosition);
    void setVehicleTarget(int vehicleId, int targetNodeId);
    void updateVehicles(float deltaTime);

    // State queries
    bool isVehicleMoving(int vehicleId) const;
    bool hasVehicleArrived(int vehicleId) const;
    std::vector<int> getVehiclesAtPosition(const Point& position, float radius = 20.0f) const;
    void coordinateVehicleMovements(); // Coordinate all vehicle movements
    
    // Advanced conflict detection and resolution
    struct VehicleConflict {
        int vehicleId1;
        int vehicleId2;
        int conflictJunctionId;
        float estimatedConflictTime;
        bool canNegotiate;
    };
    
    std::vector<VehicleConflict> detectUpcomingConflicts() const;
    bool shouldVehicleWaitForConflictResolution(int vehicleId, const std::vector<VehicleConflict>& conflicts) const;
    bool resolveConflictThroughNegotiation(const VehicleConflict& conflict);
    void preventDeadlockThroughCoordination();

    // Path management
    bool planPath(int vehicleId, int targetNodeId);
    bool replanPathIfBlocked(int vehicleId);
    void clearPath(int vehicleId);
    bool isPathBlocked(int vehicleId) const;
    bool findAlternativePath(int vehicleId);
    void handleBlockedVehicle(Auto& vehicle);

    // Getters
    const std::vector<Auto>& getVehicles() const { return vehicles; }
    size_t getVehicleCount() const { return vehicles.size(); }
    const PathSystem* getPathSystem() const { return pathSystem; }

    // Helper methods for conflict detection
    int findCommonJunctionInPaths(const Auto& vehicle1, const Auto& vehicle2) const;
    float estimateTimeToJunction(const Auto& vehicle, int junctionId) const;

private:
    void updateVehicleMovement(Auto& vehicle, float deltaTime);
    void moveVehicleAlongPath(Auto& vehicle, float deltaTime);
    bool checkCollisionRisk(const Auto& vehicle, int segmentId);
    Point interpolatePosition(const Point& start, const Point& end, float t) const;
    bool tryReserveNextSegment(Auto& vehicle);
    void releaseCurrentSegment(Auto& vehicle);

    // T-junction conflict resolution
    bool mustWaitAtWaitingNodeForBlockedPath(const Auto& vehicle);
    bool isPathAheadBlocked(const Auto& vehicle);
    int findNextJunctionInPath(const Auto& vehicle);
    int findSegmentAfterJunction(const Auto& vehicle, int junctionId);
    bool hasConflictAtJunction(int junctionId, int vehicleId);
    void moveToNearestWaitingNode(Auto& vehicle);
    int findNearestWaitingNode(int currentNodeId);
    bool findAlternativeRouteFromWaitingNode(Auto& vehicle);

    // Legacy T-junction methods (kept for compatibility)
    bool mustWaitAtWaitingNode(const Auto& vehicle);
    int findTJunctionForWaitingNode(int waitingNodeId);
    bool hasConflictAtTJunction(int tJunctionId, int vehicleId);
    bool resolveTJunctionConflict(int tJunctionId, int vehicleId, const std::vector<int>& conflictingVehicles);
    VehicleIntention getVehicleIntention(int vehicleId, int tJunctionId);
    Direction calculateDirection(const Point& from, const Point& to);
    Direction getOppositeDirection(Direction dir);
    bool tryEvasionRoute(int vehicleId, int tJunctionId);
    bool hasMinimumDistanceToOtherVehicles(const Auto& vehicle, float minDistance);
    bool canMoveWithoutViolatingDistance(const Auto& vehicle, const Point& targetPosition, float minDistance);

    PathSystem* pathSystem;
    SegmentManager* segmentManager;
    std::vector<Auto> vehicles;
    std::unordered_map<int, size_t> vehicleIdToIndex;
    int nextVehicleId;
};