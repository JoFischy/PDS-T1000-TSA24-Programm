
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "auto.h"
#include "movement_system.h"
#include <memory>
#include <map>

struct VehicleState {
    Point currentPosition;
    Direction currentDirection;
    size_t currentPathIndex;
    bool isMoving;
    float speed; // pixels per frame
    
    VehicleState() 
        : currentDirection(Direction::NORTH), currentPathIndex(0), 
          isMoving(false), speed(2.0f) {}
};

class VehicleController : public MovementInterface {
private:
    std::map<int, VehicleState> vehicleStates;
    PathSystem pathSystem;
    
    Direction calculateDirection(const Point& from, const Point& to) const;
    Point moveTowardsTarget(const Point& current, const Point& target, float speed) const;
    
public:
    VehicleController();
    
    // MovementInterface implementation
    void moveForward(int vehicleId, float distance) override;
    void turnLeft(int vehicleId) override;
    void turnRight(int vehicleId) override;
    void setPosition(int vehicleId, const Point& position, Direction direction) override;
    
    // Vehicle management
    void addVehicle(const Auto& vehicle);
    void removeVehicle(int vehicleId);
    void updateVehicle(int vehicleId, Auto& vehicle);
    void updateAllVehicles(std::vector<Auto>& vehicles);
    
    // Path management
    void createSamplePath();
    PathSystem& getPathSystem() { return pathSystem; }
    const PathSystem& getPathSystem() const { return pathSystem; }
    
    // State access
    const std::map<int, VehicleState>& getVehicleStates() const { return vehicleStates; }
    void setVehicleSpeed(int vehicleId, float speed);
    bool isVehicleMoving(int vehicleId) const;
    void stopVehicle(int vehicleId);
    void startVehicle(int vehicleId);
};

#endif
