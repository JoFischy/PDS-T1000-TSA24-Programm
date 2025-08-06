#include "renderer.h"
#include <cmath>

Renderer::Renderer(int width, int height) 
    : screenWidth(width), screenHeight(height) {
    
    // Initialize colors
    backgroundColor = WHITE;
    boundaryColor = BLACK;
    frontPointColor = RED;
    rearPointColor = BLUE;
    autoBodyColor = GRAY;
    autoIdColor = BLACK;
}

void Renderer::initialize() {
    InitWindow(screenWidth, screenHeight, "Autonomous Transporter Management");
    SetTargetFPS(60);
}

void Renderer::render(const std::vector<Auto>& autos, const Simulation& simulation) {
    BeginDrawing();
    
    ClearBackground(backgroundColor);
    
    // Draw map boundaries
    drawMap(simulation.getMapBoundaries());
    
    // Draw simulation points
    drawPoints(simulation.getFrontPoints(), simulation.getRearPoints());
    
    // Draw vehicles
    for (const Auto& auto_ : autos) {
        if (auto_.getIsValid()) {
            drawVehicle(auto_.getPosition(), auto_.getDirection(), auto_.getId(), true);
        }
    }
    
    // Draw info text
    DrawText("Autonomous Transporter Management System", 10, 10, 20, BLACK);
    DrawText("Red: Front Points | Blue: Rear Points | Gray: Vehicles", 10, 35, 16, BLACK);
    
    // Draw vehicle info with detection status
    int yOffset = 60;
    for (const Auto& auto_ : autos) {
        if (auto_.getIsValid()) {
            char info[256];
            char frontStatus = auto_.getFrontPointValid() ? 'F' : '-';
            char rearStatus = auto_.getRearPointValid() ? 'R' : '-';
            
            snprintf(info, sizeof(info), 
                "Auto %d: Pos(%.1f, %.1f) Dir: %.1f° [%c%c]", 
                auto_.getId(), 
                auto_.getPosition().x, 
                auto_.getPosition().y, 
                auto_.getDirection(),
                frontStatus,
                rearStatus);
            DrawText(info, 10, yOffset, 14, BLACK);
            yOffset += 20;
        }
    }
    
    EndDrawing();
}

void Renderer::drawMap(const std::vector<std::vector<Point>>& boundaries) {
    for (const auto& boundary : boundaries) {
        if (boundary.size() < 2) continue;
        
        for (size_t i = 0; i < boundary.size() - 1; i++) {
            DrawLineEx(
                {boundary[i].x, boundary[i].y},
                {boundary[i + 1].x, boundary[i + 1].y},
                4.0f,
                boundaryColor
            );
        }
    }
}

void Renderer::drawPoints(const std::vector<Point>& frontPoints, const std::vector<Point>& rearPoints) {
    // Draw front points
    for (const Point& point : frontPoints) {
        DrawCircle(static_cast<int>(point.x), static_cast<int>(point.y), 6, frontPointColor);
        DrawCircleLines(static_cast<int>(point.x), static_cast<int>(point.y), 8, BLACK);
    }
    
    // Draw rear points
    for (size_t i = 0; i < rearPoints.size(); i++) {
        const Point& point = rearPoints[i];
        DrawCircle(static_cast<int>(point.x), static_cast<int>(point.y), 6, rearPointColor);
        DrawCircleLines(static_cast<int>(point.x), static_cast<int>(point.y), 8, BLACK);
        
        // Draw ID number next to rear point
        char idText[8];
        snprintf(idText, sizeof(idText), "%zu", i + 1);
        DrawText(idText, static_cast<int>(point.x + 12), static_cast<int>(point.y - 8), 16, BLACK);
    }
}

void Renderer::drawVehicle(const Point& position, float direction, int id, bool isValid) {
    if (!isValid) return;
    
    float dirRad = direction * M_PI / 180.0f;
    float halfLength = Auto::getVehicleLength() / 2.0f;
    float halfWidth = Auto::getVehicleWidth() / 2.0f;
    
    // Calculate vehicle corners
    Vector2 corners[4];
    
    // Front-left, front-right, rear-right, rear-left
    float frontX = position.x + sinf(dirRad) * halfLength;
    float frontY = position.y - cosf(dirRad) * halfLength;
    float rearX = position.x - sinf(dirRad) * halfLength;
    float rearY = position.y + cosf(dirRad) * halfLength;
    
    float perpX = cosf(dirRad);
    float perpY = sinf(dirRad);
    
    corners[0] = {frontX - perpX * halfWidth, frontY - perpY * halfWidth}; // Front-left
    corners[1] = {frontX + perpX * halfWidth, frontY + perpY * halfWidth}; // Front-right
    corners[2] = {rearX + perpX * halfWidth, rearY + perpY * halfWidth};   // Rear-right
    corners[3] = {rearX - perpX * halfWidth, rearY - perpY * halfWidth};   // Rear-left
    
    // Draw vehicle body
    for (int i = 0; i < 4; i++) {
        Vector2 start = corners[i];
        Vector2 end = corners[(i + 1) % 4];
        DrawLineEx(start, end, 3.0f, autoBodyColor);
    }
    
    // Fill vehicle body with semi-transparent color
    Color fillColor = autoBodyColor;
    fillColor.a = 100;
    
    // Simple triangle fill approximation
    DrawTriangle(corners[0], corners[1], corners[2], fillColor);
    DrawTriangle(corners[0], corners[2], corners[3], fillColor);
    
    // Draw direction arrow
    Vector2 arrowStart = {position.x, position.y};
    Vector2 arrowEnd = {
        position.x + sinf(dirRad) * 40.0f,
        position.y - cosf(dirRad) * 40.0f
    };
    DrawLineEx(arrowStart, arrowEnd, 3.0f, RED);
    
    // Draw arrowhead
    float arrowHeadAngle = 0.5f;
    float arrowHeadLength = 15.0f;
    Vector2 arrowHead1 = {
        arrowEnd.x - sinf(dirRad + arrowHeadAngle) * arrowHeadLength,
        arrowEnd.y + cosf(dirRad + arrowHeadAngle) * arrowHeadLength
    };
    Vector2 arrowHead2 = {
        arrowEnd.x - sinf(dirRad - arrowHeadAngle) * arrowHeadLength,
        arrowEnd.y + cosf(dirRad - arrowHeadAngle) * arrowHeadLength
    };
    DrawLineEx(arrowEnd, arrowHead1, 2.0f, RED);
    DrawLineEx(arrowEnd, arrowHead2, 2.0f, RED);
    
    // Draw vehicle ID
    char idText[8];
    snprintf(idText, sizeof(idText), "%d", id);
    DrawText(idText, static_cast<int>(position.x - 8), static_cast<int>(position.y - 8), 20, autoIdColor);
}

bool Renderer::shouldClose() const {
    return WindowShouldClose();
}

void Renderer::cleanup() {
    CloseWindow();
}
