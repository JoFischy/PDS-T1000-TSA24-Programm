
#pragma once
#include "auto.h"
#include "path_system.h"
#include "segment_manager.h"
#include <vector>
#include <unordered_map>
#include <random>
#include <memory>

class VehicleController {
public:
    VehicleController(PathSystem* pathSys, SegmentManager* segmentMgr);
    
    // Fahrzeugverwaltung
    int addVehicle(const Point& startPosition);
    void removeVehicle(int vehicleId);
    void spawnInitialVehicles();
    void assignRandomTargetsToAllVehicles();
    
    // Bewegungssteuerung
    void updateVehicles(float deltaTime);
    void setVehicleTarget(int vehicleId, int targetNodeId);
    void setVehicleTargetNode(int vehicleId, int targetNodeId);
    void assignNewRandomTarget(int vehicleId);
    
    // Getter
    Auto* getVehicle(int vehicleId);
    const Auto* getVehicle(int vehicleId) const;
    const std::vector<Auto>& getVehicles() const { return vehicles; }
    size_t getVehicleCount() const { return vehicles.size(); }
    PathSystem* getPathSystem() const { return pathSystem; }
    
private:
    void updateVehicleMovement(Auto& vehicle, float deltaTime);
    void moveVehicleAlongSegment(Auto& vehicle, float deltaTime);
    bool planPath(int vehicleId, int targetNodeId);
    void handleIdleVehicle(Auto& vehicle);
    void handleMovingVehicle(Auto& vehicle, float deltaTime);
    void handleWaitingVehicle(Auto& vehicle);
    std::vector<int> getNextSectionFromWaitingNode(int waitingNodeId, const std::vector<int>& path);
    int generateRandomTarget(int vehicleId);
    
    PathSystem* pathSystem;
    SegmentManager* segmentManager;
    std::vector<Auto> vehicles;
    std::unordered_map<int, size_t> vehicleIdToIndex;
    int nextVehicleId;
    
    std::mt19937 randomGenerator;
    std::uniform_real_distribution<float> randomFloat;
};
