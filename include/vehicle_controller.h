#pragma once
#include "auto.h"
#include "path_system.h"
#include "segment_manager.h"
#include <vector>
#include <unordered_map>

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
    
private:
    void updateVehicleMovement(Auto& vehicle, float deltaTime);
    void moveVehicleAlongPath(Auto& vehicle, float deltaTime);
    bool checkCollisionRisk(const Auto& vehicle, int segmentId);
    Point interpolatePosition(const Point& start, const Point& end, float t) const;
    bool tryReserveNextSegment(Auto& vehicle);
    void releaseCurrentSegment(Auto& vehicle);
    
    PathSystem* pathSystem;
    SegmentManager* segmentManager;
    std::vector<Auto> vehicles;
    std::unordered_map<int, size_t> vehicleIdToIndex;
    int nextVehicleId;
};