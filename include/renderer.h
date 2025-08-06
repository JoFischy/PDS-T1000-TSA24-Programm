#ifndef RENDERER_H
#define RENDERER_H

#include "raylib.h"
#include "point.h"
#include "auto.h"
#include "movement_system.h"
#include <vector>

class Renderer {
private:
    int screenWidth;
    int screenHeight;

    // Colors
    Color backgroundColor;
    Color pointColor;
    Color selectedPointColor;
    Color autoColor;
    Color uiColor;

public:
    Renderer(int width, int height);

    void initialize();
    void cleanup();
    bool shouldClose();
    void render(const std::vector<Point>& points, const std::vector<Auto>& detectedAutos, float tolerance);
    void render(const std::vector<Point>& points, const std::vector<Auto>& detectedAutos, float tolerance, const PathSystem& pathSystem);
    void renderPath(const std::vector<Point>& path); // Added for path rendering

private:
    void drawPoint(const Point& point, int index, bool isSelected);
    void drawAuto(const Auto& auto_);
    void drawUI(float tolerance);
    void drawToleranceVisualization(const std::vector<Point>& points, float tolerance);
    void drawVehicleInfo(const std::vector<Auto>& detectedAutos);
    void drawPathSegment(const Point& start, const Point& end); // Helper for drawing path segments
    void drawPath(const PathSystem& pathSystem); // Added for path rendering
};

#endif