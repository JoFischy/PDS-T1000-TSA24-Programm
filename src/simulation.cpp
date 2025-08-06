#include "simulation.h"
#include "auto.h"
#include <cmath>
#include <chrono>

Simulation::Simulation() : rng(std::chrono::steady_clock::now().time_since_epoch().count()) {
    // Initialize exactly 4 vehicles with front and rear identification points
    frontPoints.resize(4);
    rearPoints.resize(4);
    velocities.resize(4);
    directions.resize(4);
    turnRates.resize(4);
    
    initializeMap();
    
    // Initialize 4 vehicles in the center area of the screen
    std::uniform_real_distribution<float> velDist(25.0f, 45.0f);
    std::uniform_real_distribution<float> dirDist(0.0f, 360.0f);
    std::uniform_real_distribution<float> turnDist(-15.0f, 15.0f);
    std::uniform_real_distribution<float> xDist(400.0f, 1520.0f);  // More centered
    std::uniform_real_distribution<float> yDist(300.0f, 900.0f);   // More centered
    
    for (int i = 0; i < 4; i++) {
        // Create vehicle center position
        Point center;
        center.x = xDist(rng);
        center.y = yDist(rng);
        
        directions[i] = dirDist(rng);
        velocities[i] = velDist(rng);
        turnRates[i] = turnDist(rng);
        
        // Calculate front and rear points with exact vehicle length
        float dirRad = directions[i] * M_PI / 180.0f;
        float halfLength = Auto::getVehicleLength() / 2.0f;
        
        frontPoints[i].x = center.x + sinf(dirRad) * halfLength;
        frontPoints[i].y = center.y - cosf(dirRad) * halfLength;
        
        rearPoints[i].x = center.x - sinf(dirRad) * halfLength;
        rearPoints[i].y = center.y + cosf(dirRad) * halfLength;
    }
}

void Simulation::initializeMap() {
    // Create a more detailed map with wider roads for better vehicle movement
    // Outer boundaries
    mapBoundaries.push_back({
        {30, 30}, {1890, 30}, {1890, 1170}, {30, 1170}, {30, 30}
    });
    
    // Create road system with wider paths (200+ pixels wide for easy driving)
    
    // Horizontal road barriers (top section)
    mapBoundaries.push_back({
        {150, 150}, {150, 250}, {450, 250}, {450, 150}, {150, 150}
    });
    mapBoundaries.push_back({
        {650, 150}, {650, 250}, {950, 250}, {950, 150}, {650, 150}
    });
    mapBoundaries.push_back({
        {1150, 150}, {1150, 250}, {1450, 250}, {1450, 150}, {1150, 150}
    });
    mapBoundaries.push_back({
        {1650, 150}, {1650, 250}, {1850, 250}, {1850, 150}, {1650, 150}
    });
    
    // Horizontal road barriers (middle section)
    mapBoundaries.push_back({
        {150, 450}, {150, 550}, {450, 550}, {450, 450}, {150, 450}
    });
    mapBoundaries.push_back({
        {650, 450}, {650, 550}, {950, 550}, {950, 450}, {650, 450}
    });
    mapBoundaries.push_back({
        {1150, 450}, {1150, 550}, {1450, 550}, {1450, 450}, {1150, 450}
    });
    mapBoundaries.push_back({
        {1650, 450}, {1650, 550}, {1850, 550}, {1850, 450}, {1650, 450}
    });
    
    // Horizontal road barriers (bottom section)
    mapBoundaries.push_back({
        {150, 750}, {150, 850}, {450, 850}, {450, 750}, {150, 750}
    });
    mapBoundaries.push_back({
        {650, 750}, {650, 850}, {950, 850}, {950, 750}, {650, 750}
    });
    mapBoundaries.push_back({
        {1150, 750}, {1150, 850}, {1450, 850}, {1450, 750}, {1150, 750}
    });
    mapBoundaries.push_back({
        {1650, 750}, {1650, 850}, {1850, 850}, {1850, 750}, {1650, 750}
    });
    
    // Vertical road barriers (left column)
    mapBoundaries.push_back({
        {550, 250}, {650, 250}, {650, 450}, {550, 450}, {550, 250}
    });
    mapBoundaries.push_back({
        {550, 550}, {650, 550}, {650, 750}, {550, 750}, {550, 550}
    });
    
    // Vertical road barriers (middle column)
    mapBoundaries.push_back({
        {1050, 250}, {1150, 250}, {1150, 450}, {1050, 450}, {1050, 250}
    });
    mapBoundaries.push_back({
        {1050, 550}, {1150, 550}, {1150, 750}, {1050, 750}, {1050, 550}
    });
    
    // Vertical road barriers (right column)
    mapBoundaries.push_back({
        {1550, 250}, {1650, 250}, {1650, 450}, {1550, 450}, {1550, 250}
    });
    mapBoundaries.push_back({
        {1550, 550}, {1650, 550}, {1650, 750}, {1550, 750}, {1550, 550}
    });
}

bool Simulation::isValidPosition(const Point& pos) const {
    // Check if position is within screen bounds with larger margin
    if (pos.x < 60 || pos.x > 1860 || pos.y < 60 || pos.y > 1140) {
        return false;
    }
    
    // Check collision with obstacles (everything except outer boundary)
    for (size_t i = 1; i < mapBoundaries.size(); i++) {
        const auto& boundary = mapBoundaries[i];
        if (boundary.size() < 4) continue;
        
        // Rectangular collision check with some margin for vehicle size
        float minX = boundary[0].x, maxX = boundary[0].x;
        float minY = boundary[0].y, maxY = boundary[0].y;
        
        for (const auto& point : boundary) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }
        
        // Add some margin around obstacles for vehicle clearance
        float margin = 25.0f;
        if (pos.x >= (minX - margin) && pos.x <= (maxX + margin) && 
            pos.y >= (minY - margin) && pos.y <= (maxY + margin)) {
            return false;
        }
    }
    
    return true;
}

Point Simulation::getRandomValidPosition() {
    std::uniform_real_distribution<float> xDist(150, 1770);
    std::uniform_real_distribution<float> yDist(150, 1050);
    
    Point pos;
    int attempts = 0;
    do {
        pos.x = xDist(rng);
        pos.y = yDist(rng);
        attempts++;
    } while (!isValidPosition(pos) && attempts < 100);
    
    return pos;
}

void Simulation::update(float deltaTime) {
    std::uniform_real_distribution<float> turnChangeDist(-8.0f, 8.0f);
    std::uniform_real_distribution<float> speedChangeDist(0.99f, 1.01f);
    std::uniform_real_distribution<float> randomTurnDist(-0.3f, 0.3f);
    std::uniform_real_distribution<float> pointNoiseDist(-1.5f, 1.5f);
    
    for (int i = 0; i < 4; i++) {
        // Randomly adjust movement parameters
        if (rng() % 150 == 0) {
            turnRates[i] += turnChangeDist(rng);
            turnRates[i] = std::max(-25.0f, std::min(25.0f, turnRates[i]));
        }
        
        if (rng() % 200 == 0) {
            velocities[i] *= speedChangeDist(rng);
            velocities[i] = std::max(20.0f, std::min(50.0f, velocities[i]));
        }
        
        // Add gentle random steering
        turnRates[i] += randomTurnDist(rng) * deltaTime;
        
        // Update direction
        directions[i] += turnRates[i] * deltaTime;
        if (directions[i] < 0) directions[i] += 360.0f;
        if (directions[i] >= 360.0f) directions[i] -= 360.0f;
        
        // Calculate current vehicle center from existing points
        Point center;
        if (frontPoints[i].x > 0 && rearPoints[i].x > 0) {
            center.x = (frontPoints[i].x + rearPoints[i].x) / 2.0f;
            center.y = (frontPoints[i].y + rearPoints[i].y) / 2.0f;
        } else {
            // If points are missing, use estimated center
            center.x = 960.0f; // Default to screen center
            center.y = 600.0f;
        }
        
        // Move vehicle center
        float dirRad = directions[i] * M_PI / 180.0f;
        float moveX = sinf(dirRad) * velocities[i] * deltaTime;
        float moveY = -cosf(dirRad) * velocities[i] * deltaTime;
        
        center.x += moveX;
        center.y += moveY;
        
        // Keep vehicles more towards center area
        if (center.x < 350) center.x = 1570;
        else if (center.x > 1570) center.x = 350;
        if (center.y < 250) center.y = 950;
        else if (center.y > 950) center.y = 250;
        
        // Calculate precise front and rear points with exact vehicle length
        float halfLength = Auto::getVehicleLength() / 2.0f;
        
        Point newFront, newRear;
        newFront.x = center.x + sinf(dirRad) * halfLength;
        newFront.y = center.y - cosf(dirRad) * halfLength;
        newRear.x = center.x - sinf(dirRad) * halfLength;
        newRear.y = center.y + cosf(dirRad) * halfLength;
        
        // Add minimal noise occasionally to simulate detection inaccuracy
        if (rng() % 80 == 0) {
            newFront.x += pointNoiseDist(rng);
            newFront.y += pointNoiseDist(rng);
        }
        if (rng() % 85 == 0) {
            newRear.x += pointNoiseDist(rng);
            newRear.y += pointNoiseDist(rng);
        }
        
        // Simulate detection failures (3% chance per point)
        bool frontDetected = (rng() % 33 != 0); // 97% detection rate
        bool rearDetected = (rng() % 33 != 0);
        
        // Update points
        if (frontDetected) {
            frontPoints[i] = newFront;
        } else {
            frontPoints[i].x = -1000;
            frontPoints[i].y = -1000;
        }
        
        if (rearDetected) {
            rearPoints[i] = newRear;
        } else {
            rearPoints[i].x = -1000;
            rearPoints[i].y = -1000;
        }
        
        // Restore lost points quickly (realistic recovery)
        if (frontPoints[i].x < 0 && rng() % 30 == 0) {
            frontPoints[i] = newFront;
        }
        if (rearPoints[i].x < 0 && rng() % 30 == 0) {
            rearPoints[i] = newRear;
        }
    }
    
    // Simulate front point swapping when vehicles get close (rare event)
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (frontPoints[i].x < 0 || frontPoints[j].x < 0) continue;
            
            // Calculate distance between front points
            float dx = frontPoints[i].x - frontPoints[j].x;
            float dy = frontPoints[i].y - frontPoints[j].y;
            float distance = sqrtf(dx * dx + dy * dy);
            
            // If vehicles are very close (within 80 pixels), occasionally swap front points
            if (distance < 80.0f && rng() % 300 == 0) { // Very rare event
                Point temp = frontPoints[i];
                frontPoints[i] = frontPoints[j];
                frontPoints[j] = temp;
            }
        }
    }
}
