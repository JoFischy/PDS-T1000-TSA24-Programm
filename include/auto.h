#ifndef AUTO_H
#define AUTO_H

#include "point.h"

class Auto {
private:
    int id;
    Point frontPoint;
    Point rearPoint;
    Point position;      // Midpoint between front and rear
    Point lastKnownPosition;  // Last known position for prediction
    float direction;     // Direction in degrees (0-359)
    float lastKnownDirection; // Last known direction for prediction
    bool isValid;        // Whether the auto has valid data
    bool frontPointValid; // Whether front point is currently detected
    bool rearPointValid;  // Whether rear point is currently detected
    float timeSinceLastUpdate; // Time since last valid update
    
    static const float VEHICLE_LENGTH; // Expected distance between front and rear points
    static const float TOLERANCE;      // Tolerance for point matching
    
public:
    Auto(int autoId);
    
    // Update auto with new front and rear points (can handle missing points)
    bool updatePoints(const Point& newFront, const Point& newRear);
    bool updateFrontPoint(const Point& newFront);
    bool updateRearPoint(const Point& newRear);
    
    // Update time for prediction when no points are available
    void updateTime(float deltaTime);
    
    // Getters
    int getId() const { return id; }
    Point getPosition() const { return position; }
    Point getFrontPoint() const { return frontPoint; }
    Point getRearPoint() const { return rearPoint; }
    float getDirection() const { return direction; }
    bool getIsValid() const { return isValid; }
    bool getFrontPointValid() const { return frontPointValid; }
    bool getRearPointValid() const { return rearPointValid; }
    
    // Calculate position and direction from available points
    void calculatePositionAndDirection();
    
    // Predict position when points are missing
    void predictPosition(float deltaTime);
    
    // Check if given points could belong to this auto
    bool couldBelongToAuto(const Point& front, const Point& rear) const;
    
    // Get vehicle dimensions
    static float getVehicleLength() { return VEHICLE_LENGTH; }
    static float getVehicleWidth() { return 107.0f; }
};

#endif
