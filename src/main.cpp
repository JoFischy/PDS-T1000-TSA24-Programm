#include "auto.h"
#include "simulation.h"
#include "renderer.h"
#include <vector>
#include <algorithm>

class TransporterManager {
private:
    std::vector<Auto> autos;
    Simulation simulation;
    Renderer renderer;
    
    // Match points to vehicles based on proximity and vehicle constraints
    void matchPointsToVehicles(const std::vector<Point>& frontPoints, 
                              const std::vector<Point>& rearPoints) {
        // Filter out invalid points (detection failures)
        std::vector<Point> validFrontPoints, validRearPoints;
        
        for (const Point& front : frontPoints) {
            if (front.x > 0 && front.y > 0) { // Valid detection
                validFrontPoints.push_back(front);
            }
        }
        
        for (const Point& rear : rearPoints) {
            if (rear.x > 0 && rear.y > 0) { // Valid detection
                validRearPoints.push_back(rear);
            }
        }
        
        // Create a list of potential vehicle configurations
        std::vector<std::pair<Point, Point>> potentialVehicles;
        
        // Try all combinations of front and rear points
        for (const Point& front : validFrontPoints) {
            for (const Point& rear : validRearPoints) {
                float distance = front.distanceTo(rear);
                // Check if distance is approximately correct for a vehicle
                if (std::abs(distance - Auto::getVehicleLength()) <= 50.0f) {
                    potentialVehicles.push_back({front, rear});
                }
            }
        }
        
        // First pass: Try to match potential vehicles to existing autos
        std::vector<bool> usedVehicles(potentialVehicles.size(), false);
        std::vector<bool> autoUpdated(autos.size(), false);
        
        for (size_t autoIdx = 0; autoIdx < autos.size(); autoIdx++) {
            Auto& auto_ = autos[autoIdx];
            float bestScore = std::numeric_limits<float>::max();
            int bestMatch = -1;
            
            for (size_t i = 0; i < potentialVehicles.size(); i++) {
                if (usedVehicles[i]) continue;
                
                const Point& front = potentialVehicles[i].first;
                const Point& rear = potentialVehicles[i].second;
                
                // Calculate score based on distance from previous position
                float score = std::numeric_limits<float>::max();
                
                if (auto_.getIsValid()) {
                    // Prefer vehicles close to last known position
                    float frontDist = auto_.getFrontPoint().distanceTo(front);
                    float rearDist = auto_.getRearPoint().distanceTo(rear);
                    
                    // Also try swapped assignment (in case points got mixed up)
                    float frontDistSwapped = auto_.getFrontPoint().distanceTo(rear);
                    float rearDistSwapped = auto_.getRearPoint().distanceTo(front);
                    
                    float normalScore = frontDist + rearDist;
                    float swappedScore = frontDistSwapped + rearDistSwapped;
                    
                    if (swappedScore < normalScore && swappedScore < 100.0f) {
                        // Use swapped assignment
                        score = swappedScore;
                    } else if (normalScore < 150.0f) {
                        // Use normal assignment
                        score = normalScore;
                    }
                } else {
                    // No previous position, accept any valid configuration
                    score = 1.0f;
                }
                
                if (score < bestScore && score < 200.0f) {
                    bestScore = score;
                    bestMatch = static_cast<int>(i);
                }
            }
            
            // Update auto with best match
            if (bestMatch != -1) {
                const Point& front = potentialVehicles[bestMatch].first;
                const Point& rear = potentialVehicles[bestMatch].second;
                
                // Check if we should swap points based on previous position
                if (auto_.getIsValid()) {
                    float frontDist = auto_.getFrontPoint().distanceTo(front);
                    float rearDist = auto_.getRearPoint().distanceTo(rear);
                    float frontDistSwapped = auto_.getFrontPoint().distanceTo(rear);
                    float rearDistSwapped = auto_.getRearPoint().distanceTo(front);
                    
                    if ((frontDistSwapped + rearDistSwapped) < (frontDist + rearDist)) {
                        // Swap points
                        auto_.updatePoints(rear, front);
                    } else {
                        auto_.updatePoints(front, rear);
                    }
                } else {
                    auto_.updatePoints(front, rear);
                }
                
                usedVehicles[bestMatch] = true;
                autoUpdated[autoIdx] = true;
            }
        }
        
        // Second pass: For unused potential vehicles, assign to uninitialized autos
        for (size_t i = 0; i < potentialVehicles.size(); i++) {
            if (usedVehicles[i]) continue;
            
            for (size_t autoIdx = 0; autoIdx < autos.size(); autoIdx++) {
                if (autoUpdated[autoIdx]) continue;
                
                Auto& auto_ = autos[autoIdx];
                const Point& front = potentialVehicles[i].first;
                const Point& rear = potentialVehicles[i].second;
                
                if (auto_.updatePoints(front, rear)) {
                    usedVehicles[i] = true;
                    autoUpdated[autoIdx] = true;
                    break;
                }
            }
        }
    }
    
public:
    TransporterManager() : renderer(1920, 1200) {
        // Initialize 4 autos - they will only be displayed when valid point pairs are found
        for (int i = 1; i <= 4; i++) {
            autos.emplace_back(i);
        }
    }
    
    void run() {
        renderer.initialize();
        
        float lastTime = 0.0f;
        
        while (!renderer.shouldClose()) {
            float currentTime = GetTime();
            float deltaTime = currentTime - lastTime;
            lastTime = currentTime;
            
            // Update simulation (only simulates points, not complete vehicles)
            simulation.update(deltaTime);
            
            // Get simulated points
            std::vector<Point> frontPoints = simulation.getFrontPoints();
            std::vector<Point> rearPoints = simulation.getRearPoints();
            
            // Reset all autos to invalid state at start of each frame
            // They will only become valid if matching point pairs are found
            
            // Update time for all autos (for prediction when points are missing)
            for (Auto& auto_ : autos) {
                auto_.updateTime(deltaTime);
            }
            
            // Match points to vehicles - only shows vehicles where valid point pairs exist
            matchPointsToVehicles(frontPoints, rearPoints);
            
            // Render everything
            renderer.render(autos, simulation);
        }
        
        renderer.cleanup();
    }
};

int main() {
    TransporterManager manager;
    manager.run();
    return 0;
}
