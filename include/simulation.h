#ifndef SIMULATION_H
#define SIMULATION_H

#include "point.h"
#include "auto.h"
#include <vector>
#include <random>

class Simulation {
private:
    std::vector<Point> frontPoints;
    std::vector<Point> rearPoints;
    std::vector<float> velocities;
    std::vector<float> directions;
    std::vector<float> turnRates;
    std::mt19937 rng;
    
    // Map boundaries and paths
    std::vector<std::vector<Point>> mapBoundaries;
    
    // Check if a point is within valid driving area
    bool isValidPosition(const Point& pos) const;
    
    // Get random valid starting position
    Point getRandomValidPosition();
    
    // Initialize map boundaries
    void initializeMap();
    

    
public:
    Simulation();
    
    // Update simulation step
    void update(float deltaTime);
    
    // Get current simulated points
    std::vector<Point> getFrontPoints() const { return frontPoints; }
    std::vector<Point> getRearPoints() const { return rearPoints; }
    
    // Get map boundaries for rendering
    const std::vector<std::vector<Point>>& getMapBoundaries() const { return mapBoundaries; }
};

#endif
