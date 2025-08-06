#include "renderer.h"
#include <cstdio>
#include <cmath> // For M_PI, cosf, sinf

Renderer::Renderer(int width, int height)
    : screenWidth(width), screenHeight(height) {

    // Initialize colors
    backgroundColor = WHITE;
    pointColor = BLUE;
    selectedPointColor = RED;
    autoColor = GREEN;
    uiColor = BLACK;
}

void Renderer::initialize() {
    InitWindow(screenWidth, screenHeight, "Point Vehicle Detection System");
    SetTargetFPS(60);
}

void Renderer::cleanup() {
    CloseWindow();
}

bool Renderer::shouldClose() {
    return WindowShouldClose();
}

void Renderer::render(const std::vector<Point>& points, const std::vector<Auto>& detectedAutos, float tolerance) {
    BeginDrawing();

    ClearBackground(backgroundColor);

    // Draw detected vehicles first (so they appear behind points)
    for (const Auto& auto_ : detectedAutos) {
        drawAuto(auto_);
    }

    // Draw all points
    for (size_t i = 0; i < points.size(); i++) {
        bool isSelected = points[i].isDragging;
        drawPoint(points[i], static_cast<int>(i), isSelected);
    }

    // Draw UI
    drawUI(tolerance);

    EndDrawing();
}

void Renderer::drawPoint(const Point& point, int index, bool isSelected) {
    Color color = isSelected ? selectedPointColor : pointColor;

    // Draw point circle
    DrawCircle(static_cast<int>(point.x), static_cast<int>(point.y), 8, color);
    DrawCircleLines(static_cast<int>(point.x), static_cast<int>(point.y), 8, BLACK);

    // Draw point number
    char text[8];
    snprintf(text, sizeof(text), "%d", index + 1);
    DrawText(text, static_cast<int>(point.x + 12), static_cast<int>(point.y - 8), 16, BLACK);
}

void Renderer::drawAuto(const Auto& auto_) {
    if (!auto_.isValid()) return;

    Point idPoint = auto_.getIdentificationPoint();
    Point frontPoint = auto_.getFrontPoint();
    Point center = auto_.getCenter();

    // Draw line between the two points
    DrawLineEx({idPoint.x, idPoint.y}, {frontPoint.x, frontPoint.y}, 4.0f, autoColor);

    // Draw center point
    DrawCircle(static_cast<int>(center.x), static_cast<int>(center.y), 6, autoColor);

    // Draw direction arrow from identification to front point
    // Calculate direction vector from idPoint to frontPoint
    Vector2 directionVector = {frontPoint.x - idPoint.x, frontPoint.y - idPoint.y};
    // Normalize the direction vector to get a unit vector for direction calculation
    float length = sqrtf(directionVector.x * directionVector.x + directionVector.y * directionVector.y);
    if (length > 0) {
        directionVector.x /= length;
        directionVector.y /= length;
    } else {
        // If points are coincident, use a default direction or skip drawing arrow
        directionVector = {1.0f, 0.0f}; // Default to right if points are the same
    }

    float arrowLength = 25.0f;
    Point arrowEnd;
    arrowEnd.x = idPoint.x + directionVector.x * arrowLength;
    arrowEnd.y = idPoint.y + directionVector.y * arrowLength;

    DrawLineEx({idPoint.x, idPoint.y}, {arrowEnd.x, arrowEnd.y}, 3.0f, autoColor);

    // Draw arrowhead
    float headLength = 8.0f;
    float headAngle = 30.0f * M_PI / 180.0f;

    // Calculate the angle of the direction vector
    float dirRad = atan2f(directionVector.y, directionVector.x);

    Point head1, head2;
    head1.x = arrowEnd.x - cosf(dirRad - headAngle) * headLength;
    head1.y = arrowEnd.y - sinf(dirRad - headAngle) * headLength;
    head2.x = arrowEnd.x - cosf(dirRad + headAngle) * headLength;
    head2.y = arrowEnd.y - sinf(dirRad + headAngle) * headLength;

    DrawLineEx({arrowEnd.x, arrowEnd.y}, {head1.x, head1.y}, 2.0f, autoColor);
    DrawLineEx({arrowEnd.x, arrowEnd.y}, {head2.x, head2.y}, 2.0f, autoColor);
}

void Renderer::drawUI(float tolerance) {
    // Title
    DrawText("Point Vehicle Detection System", 10, 10, 20, uiColor);

    // Instructions
    DrawText("Drag points to move them around", 10, 35, 16, uiColor);
    DrawText("Use +/- keys to adjust tolerance", 10, 55, 16, uiColor);

    // Tolerance display
    char toleranceText[64];
    snprintf(toleranceText, sizeof(toleranceText), "Tolerance: %.1f pixels", tolerance);
    DrawText(toleranceText, 10, 80, 16, uiColor);

    // Legend
    DrawText("Blue: Individual Points", 10, 110, 14, uiColor);
    DrawText("Red: Selected Point", 10, 125, 14, uiColor);
    DrawText("Green: Detected Vehicle", 10, 140, 14, uiColor);
}