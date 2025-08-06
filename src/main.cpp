
/*
 * PDS-T1000 Vehicle Path Following System
 * 
 * This program creates 4 fixed vehicles that follow a predefined rectangular path.
 * 
 * Controls:
 * - ESC: Exit program
 * - P: Toggle path display on/off
 * - R: Recreate sample path
 * - UP: Increase all vehicle speeds to 3.0
 * - DOWN: Decrease all vehicle speeds to 1.0
 * - SPACE: Start/Stop all vehicles
 * 
 * Features:
 * - 4 vehicles positioned in a square formation
 * - Each vehicle follows the same rectangular path
 * - Vehicle positions and directions are updated based on path logic
 * - No draggable points - vehicles move automatically along the path
 */

#include "point.h"
#include "auto.h"
#include "renderer.h"
#include "vehicle_controller.h"
#include <vector>
#include <algorithm>

class PointManager {
private:
    std::vector<Auto> vehicles; // Changed from points to vehicles
    Renderer renderer;
    VehicleController vehicleController;
    float tolerance;
    bool pathDisplayEnabled;

    void initializeVehicles() {
        vehicles.clear();
        
        // Create 4 vehicles with fixed center points and directions
        float centerX = 960.0f;
        float centerY = 600.0f;
        float spacing = 200.0f;
        
        // Vehicle positions in a square formation
        std::vector<Point> vehicleCenters = {
            Point(centerX - spacing, centerY - spacing), // Top-left
            Point(centerX + spacing, centerY - spacing), // Top-right
            Point(centerX + spacing, centerY + spacing), // Bottom-right
            Point(centerX - spacing, centerY + spacing)  // Bottom-left
        };
        
        // Initial directions for each vehicle
        std::vector<Direction> initialDirections = {
            Direction::EAST,  // Top-left vehicle faces east
            Direction::SOUTH, // Top-right vehicle faces south
            Direction::WEST,  // Bottom-right vehicle faces west
            Direction::NORTH  // Bottom-left vehicle faces north
        };
        
        // Create vehicles with center points and directions
        for (int i = 0; i < 4; i++) {
            Point center = vehicleCenters[i];
            Direction dir = initialDirections[i];
            
            // Calculate front and identification points based on center and direction
            float offsetDistance = 20.0f; // Distance from center to front/back points
            Point frontPoint, idPoint;
            
            switch (dir) {
                case Direction::NORTH:
                    frontPoint = Point(center.x, center.y - offsetDistance);
                    idPoint = Point(center.x, center.y + offsetDistance);
                    break;
                case Direction::EAST:
                    frontPoint = Point(center.x + offsetDistance, center.y);
                    idPoint = Point(center.x - offsetDistance, center.y);
                    break;
                case Direction::SOUTH:
                    frontPoint = Point(center.x, center.y + offsetDistance);
                    idPoint = Point(center.x, center.y - offsetDistance);
                    break;
                case Direction::WEST:
                    frontPoint = Point(center.x - offsetDistance, center.y);
                    idPoint = Point(center.x + offsetDistance, center.y);
                    break;
            }
            
            // Create vehicle with calculated points
            Auto vehicle(idPoint, frontPoint);
            vehicles.push_back(vehicle);
            
            // Register vehicle with controller and set initial position
            vehicleController.addVehicle(vehicle);
            vehicleController.setPosition(vehicle.getId(), center, dir);
            vehicleController.setVehicleSpeed(vehicle.getId(), 2.0f); // Set initial speed
        }
    }

    void updateVehicles() {
        // Update all vehicle positions based on path following logic
        vehicleController.updateAllVehicles(vehicles);
        
        // Update vehicle representations based on controller state
        for (auto& vehicle : vehicles) {
            // The vehicle controller updates the internal state,
            // we just need to make sure our vehicle objects reflect the current positions
            // This is handled by the updateAllVehicles call above
        }
    }

    void updateTolerance() {
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
            tolerance += 10.0f;
            if (tolerance > 300.0f) tolerance = 300.0f;
        }

        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
            tolerance -= 10.0f;
            if (tolerance < 10.0f) tolerance = 10.0f;
        }
    }

    void updateControls() {
        if (IsKeyPressed(KEY_P)) {
            pathDisplayEnabled = !pathDisplayEnabled;
        }
        
        if (IsKeyPressed(KEY_R)) {
            vehicleController.createSamplePath();
        }
        
        // Speed controls for all vehicles
        if (IsKeyPressed(KEY_UP)) {
            for (const auto& vehicle : vehicles) {
                vehicleController.setVehicleSpeed(vehicle.getId(), 3.0f);
            }
        }
        
        if (IsKeyPressed(KEY_DOWN)) {
            for (const auto& vehicle : vehicles) {
                vehicleController.setVehicleSpeed(vehicle.getId(), 1.0f);
            }
        }
        
        // Start/Stop movement
        if (IsKeyPressed(KEY_SPACE)) {
            for (const auto& vehicle : vehicles) {
                bool isMoving = vehicleController.isVehicleMoving(vehicle.getId());
                if (isMoving) {
                    vehicleController.stopVehicle(vehicle.getId());
                } else {
                    vehicleController.startVehicle(vehicle.getId());
                }
            }
        }
    }

public:
    PointManager() : renderer(1920, 1200), tolerance(100.0f), pathDisplayEnabled(true) {
        initializeVehicles();
        vehicleController.createSamplePath();
    }

    void run() {
        renderer.initialize();

        while (!renderer.shouldClose()) {
            updateVehicles();
            updateTolerance();
            updateControls();
            
            // Create empty points vector since we don't use draggable points anymore
            std::vector<Point> emptyPoints;
            
            // Render vehicles and path
            if (pathDisplayEnabled) {
                renderer.render(emptyPoints, vehicles, tolerance, vehicleController.getPathSystem());
            } else {
                renderer.render(emptyPoints, vehicles, tolerance);
            }

            if (IsKeyPressed(KEY_ESCAPE)) {
                break;
            }
        }

        renderer.cleanup();
    }
};

int main() {
    PointManager manager;
    manager.run();
    return 0;
}
