#ifndef RENDERER_H
#define RENDERER_H

#include "auto.h"
#include "simulation.h"
#include <vector>
#include <raylib.h>

class Renderer {
private:
    int screenWidth;
    int screenHeight;
    
    // Colors
    Color backgroundColor;
    Color boundaryColor;
    Color frontPointColor;
    Color rearPointColor;
    Color autoBodyColor;
    Color autoIdColor;
    
    // Draw a vehicle at given position and direction
    void drawVehicle(const Point& position, float direction, int id, bool isValid);
    
    // Draw map boundaries
    void drawMap(const std::vector<std::vector<Point>>& boundaries);
    
    // Draw points
    void drawPoints(const std::vector<Point>& frontPoints, const std::vector<Point>& rearPoints);
    
public:
    Renderer(int width, int height);
    
    // Initialize rendering system
    void initialize();
    
    // Render complete frame
    void render(const std::vector<Auto>& autos, const Simulation& simulation);
    
    // Check if should close
    bool shouldClose() const;
    
    // Cleanup
    void cleanup();
};

#endif
