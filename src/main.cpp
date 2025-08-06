#include "point.h"
#include "auto.h"
#include "renderer.h"
#include <vector>
#include <algorithm>

class PointManager {
private:
    std::vector<Point> points;
    std::vector<Auto> detectedAutos;
    Renderer renderer;
    float tolerance;
    int draggedPointIndex;

    void initializePoints() {
        // Initialize 8 points in a grid pattern
        points.clear();
        points.resize(8);

        float centerX = 960.0f;
        float centerY = 600.0f;
        float spacing = 150.0f;

        // Arrange points in a 4x2 grid pattern
        // Alternate between identification and front points
        for (int i = 0; i < 8; i++) {
            int row = i / 4;
            int col = i % 4;

            points[i].x = centerX - (3.0f * spacing / 2.0f) + col * spacing;
            points[i].y = centerY - spacing / 2.0f + row * spacing;
            points[i].type = (i % 2 == 0) ? PointType::IDENTIFICATION : PointType::FRONT;
        }
    }

    void updateDragging() {
        Vector2 mousePos = GetMousePosition();
        bool mousePressed = IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
        bool mouseDown = IsMouseButtonDown(MOUSE_LEFT_BUTTON);
        bool mouseReleased = IsMouseButtonReleased(MOUSE_LEFT_BUTTON);

        if (mousePressed) {
            // Check if mouse is over any point
            draggedPointIndex = -1;
            for (size_t i = 0; i < points.size(); i++) {
                if (points[i].isMouseOver(mousePos.x, mousePos.y, 12.0f)) {
                    draggedPointIndex = static_cast<int>(i);
                    points[i].isDragging = true;
                    break;
                }
            }
        }

        if (mouseDown && draggedPointIndex >= 0) {
            // Update dragged point position
            points[draggedPointIndex].x = mousePos.x;
            points[draggedPointIndex].y = mousePos.y;
        }

        if (mouseReleased) {
            // Stop dragging
            for (auto& point : points) {
                point.isDragging = false;
            }
            draggedPointIndex = -1;
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

    void detectVehicles() {
        detectedAutos.clear();

        // Check all pairs of points (identification + front)
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = i + 1; j < points.size(); j++) {
                // Only pair identification points with front points
                if (points[i].type != points[j].type) {
                    float distance = points[i].distanceTo(points[j]);

                    // If two points are within tolerance, create a vehicle
                    if (distance <= tolerance) {
                        // Ensure identification point comes first
                        if (points[i].type == PointType::IDENTIFICATION) {
                            detectedAutos.emplace_back(points[i], points[j]);
                        } else {
                            detectedAutos.emplace_back(points[j], points[i]);
                        }
                    }
                }
            }
        }
    }

public:
    PointManager() : renderer(1920, 1200), tolerance(100.0f), draggedPointIndex(-1) {
        initializePoints();
    }

    void run() {
        renderer.initialize();

        while (!renderer.shouldClose()) {
            updateDragging();
            updateTolerance();
            detectVehicles();

            renderer.render(points, detectedAutos, tolerance);

            // ESC to exit
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