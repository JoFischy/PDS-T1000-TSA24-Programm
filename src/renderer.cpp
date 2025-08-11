#include "renderer.h"
#include <string>
#include <sstream>
#include <iostream>

// Static color definitions
const Color Renderer::NODE_COLOR = {100, 150, 255, 255};
const Color Renderer::SEGMENT_COLOR = {200, 200, 200, 255};
const Color Renderer::OCCUPIED_SEGMENT_COLOR = {255, 100, 100, 255};
const Color Renderer::VEHICLE_COLOR = {255, 200, 50, 255};
const Color Renderer::INTERSECTION_COLOR = {255, 50, 150, 255};
const Color Renderer::UI_BACKGROUND_COLOR = {30, 30, 30, 200};
const Color Renderer::UI_TEXT_COLOR = {255, 255, 255, 255};
const Color Renderer::PICKER_COLOR = {0, 255, 0, 255};

Renderer::Renderer() 
    : hasBackgroundImage(false), showNodes(true), showSegments(true), 
      showIntersections(true), showVehicleIds(false), showDebugInfo(false),
      windowWidth(0), windowHeight(0), isInitialized(false),
      coordinatePickerEnabled(false), pickerPosition(100, 100), 
      isDraggingPicker(false), pickerRadius(15.0f),
      offset({0, 0}), scale(1.0f) {

    camera.target = {0, 0};
    camera.offset = {0, 0};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
}

Renderer::~Renderer() {
    cleanup();
}

bool Renderer::initialize(int windowWidth, int windowHeight, const char* title) {
    this->windowWidth = windowWidth;
    this->windowHeight = windowHeight;

    InitWindow(windowWidth, windowHeight, title);
    SetTargetFPS(60);

    // Initialize camera
    camera.target = {0, 0};
    camera.offset = {windowWidth / 2.0f, windowHeight / 2.0f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    isInitialized = true;
    return true;
}

void Renderer::cleanup() {
    if (hasBackgroundImage) {
        UnloadTexture(backgroundTexture);
        hasBackgroundImage = false;
    }

    if (isInitialized) {
        CloseWindow();
        isInitialized = false;
    }
}

bool Renderer::loadBackgroundImage(const char* imagePath) {
    if (hasBackgroundImage) {
        UnloadTexture(backgroundTexture);
    }

    backgroundTexture = LoadTexture(imagePath);
    hasBackgroundImage = (backgroundTexture.id != 0);

    if (hasBackgroundImage) {
        // Center camera on image
        camera.target = {backgroundTexture.width / 2.0f, backgroundTexture.height / 2.0f};
    }

    return hasBackgroundImage;
}

void Renderer::beginFrame() {
    BeginDrawing();
    ClearBackground(DARKGRAY);
    BeginMode2D(camera);
}

void Renderer::endFrame() {
    // Render coordinate picker in world space
    if (coordinatePickerEnabled) {
        renderCoordinatePicker();
    }

    EndMode2D();
    renderUI();
    EndDrawing();
}

void Renderer::renderBackground() {
    if (hasBackgroundImage) {
        DrawTexture(backgroundTexture, 0, 0, WHITE);
    }
}

void Renderer::renderPathSystem(const PathSystem& pathSystem) {
    // Render segments first (so they appear behind nodes)
    if (showSegments) {
        for (const auto& segment : pathSystem.getSegments()) {
            renderSegment(segment, pathSystem);
        }
    }

    // Render nodes
    if (showNodes) {
        for (const auto& node : pathSystem.getNodes()) {
            renderNode(node);
        }
    }
}

void Renderer::renderVehicles(const VehicleController& vehicleController) {
    const PathSystem* pathSystem = vehicleController.getPathSystem();
    if (!pathSystem) return;

    // First render all routes
    for (const auto& vehicle : vehicleController.getVehicles()) {
        renderVehicleRoute(vehicle, vehicleController, *pathSystem);
    }

    // Then render all vehicles on top
    for (const auto& vehicle : vehicleController.getVehicles()) {
        renderVehicle(vehicle);
    }
}

void Renderer::renderDebugInfo(const PathDetector& pathDetector, const PathSystem& pathSystem) {
    if (!showDebugInfo) return;

    // Render detected intersections
    if (showIntersections) {
        for (const auto& intersection : pathDetector.getIntersections()) {
            renderIntersection(intersection);
        }
    }
}

void Renderer::renderUI() {
    // Draw UI background
    DrawRectangle(10, 10, 250, 280, UI_BACKGROUND_COLOR);

    // Draw text information
    int yOffset = 20;
    DrawText("Factory Vehicle Simulation", 15, yOffset, 16, UI_TEXT_COLOR);
    yOffset += 25;

    DrawText("Controls:", 15, yOffset, 14, UI_TEXT_COLOR);
    yOffset += 20;
    DrawText("WASD - Move camera", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("Mouse wheel - Zoom", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("Space - Pause/Resume", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("R - Reset camera", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("P - Toggle coord picker", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 20;

    DrawText("Toggle displays:", 15, yOffset, 14, UI_TEXT_COLOR);
    yOffset += 20;
    DrawText("1 - Nodes", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("2 - Segments", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("3 - Intersections", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("4 - Vehicle IDs", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("5 - Debug info", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 20;

    // Show coordinate picker info
    if (coordinatePickerEnabled) {
        DrawText("Coordinate Picker:", 15, yOffset, 14, GREEN);
        yOffset += 20;
        std::string coordText = "X: " + std::to_string((int)pickerPosition.x) + 
                               ", Y: " + std::to_string((int)pickerPosition.y);
        DrawText(coordText.c_str(), 15, yOffset, 12, GREEN);
        yOffset += 15;
        DrawText("Drag green circle to move", 15, yOffset, 10, UI_TEXT_COLOR);
    }
}

void Renderer::updateCamera() {
    // Update coordinate picker first
    if (coordinatePickerEnabled) {
        updateCoordinatePicker();
    }

    // Camera movement with WASD
    float cameraSpeed = 200.0f / camera.zoom;

    if (IsKeyDown(KEY_W)) camera.target.y -= cameraSpeed * GetFrameTime();
    if (IsKeyDown(KEY_S)) camera.target.y += cameraSpeed * GetFrameTime();
    if (IsKeyDown(KEY_A)) camera.target.x -= cameraSpeed * GetFrameTime();
    if (IsKeyDown(KEY_D)) camera.target.x += cameraSpeed * GetFrameTime();

    // Zoom with mouse wheel
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        camera.zoom += wheel * 0.1f * camera.zoom;
        if (camera.zoom < 0.1f) camera.zoom = 0.1f;
        if (camera.zoom > 5.0f) camera.zoom = 5.0f;
    }

    // Reset camera
    if (IsKeyPressed(KEY_R)) {
        resetCamera();
    }

    // Toggle coordinate picker
    if (IsKeyPressed(KEY_P)) {
        coordinatePickerEnabled = !coordinatePickerEnabled;
    }

    // Toggle display options
    if (IsKeyPressed(KEY_ONE)) showNodes = !showNodes;
    if (IsKeyPressed(KEY_TWO)) showSegments = !showSegments;
    if (IsKeyPressed(KEY_THREE)) showIntersections = !showIntersections;
    if (IsKeyPressed(KEY_FOUR)) showVehicleIds = !showVehicleIds;
    if (IsKeyPressed(KEY_FIVE)) showDebugInfo = !showDebugInfo;
}

void Renderer::zoomIn() {
    camera.zoom *= 1.2f;
    if (camera.zoom > 5.0f) camera.zoom = 5.0f;
}

void Renderer::zoomOut() {
    camera.zoom /= 1.2f;
    if (camera.zoom < 0.1f) camera.zoom = 0.1f;
}

void Renderer::resetCamera() {
    if (hasBackgroundImage) {
        camera.target = {backgroundTexture.width / 2.0f, backgroundTexture.height / 2.0f};
    } else {
        camera.target = {0, 0};
    }
    camera.zoom = 1.0f;
}

Point Renderer::screenToWorld(const Point& screenPos) const {
    Vector2 worldPos = GetScreenToWorld2D({screenPos.x, screenPos.y}, camera);
    return Point(worldPos.x, worldPos.y);
}

Point Renderer::worldToScreen(const Point& worldPos) const {
    Vector2 screenPos = GetWorldToScreen2D({worldPos.x, worldPos.y}, camera);
    return Point(screenPos.x, screenPos.y);
}

bool Renderer::shouldClose() const {
    return WindowShouldClose();
}

void Renderer::setCameraTarget(float x, float y) {
    camera.target = Vector2{x, y};
}

Vector2 Renderer::screenToWorld(Vector2 screenPos) const {
    return GetScreenToWorld2D(screenPos, camera);
}

void Renderer::renderNode(const PathNode& node) {
    // Make nodes larger and more visible
    float nodeRadius = 15.0f / camera.zoom;
    if (nodeRadius < 8.0f) nodeRadius = 8.0f; // Minimum size
    DrawCircle(node.position.x, node.position.y, nodeRadius, NODE_COLOR);
}

void Renderer::renderSegment(const PathSegment& segment, const PathSystem& pathSystem) {
    const PathNode* startNode = pathSystem.getNode(segment.startNodeId);
    const PathNode* endNode = pathSystem.getNode(segment.endNodeId);

    if (!startNode || !endNode) return;

    Color segmentColor = segment.isOccupied ? OCCUPIED_SEGMENT_COLOR : SEGMENT_COLOR;
    float thickness = 6.0f / camera.zoom;
    if (thickness < 3.0f) thickness = 3.0f; // Minimum thickness

    DrawLineEx({startNode->position.x, startNode->position.y},
               {endNode->position.x, endNode->position.y},
               thickness, segmentColor);
}

void Renderer::renderVehicle(const Auto& vehicle) {
    float radius = vehicle.size / camera.zoom;
    if (radius < 8.0f) radius = 8.0f; // Minimum size

    // Different colors based on vehicle state
    Color vehicleColor = VEHICLE_COLOR;
    switch (vehicle.state) {
        case VehicleState::MOVING: vehicleColor = GREEN; break;
        case VehicleState::WAITING: vehicleColor = ORANGE; break;
        case VehicleState::ARRIVED: vehicleColor = BLUE; break;
        default: vehicleColor = VEHICLE_COLOR; break;
    }

    DrawCircle(vehicle.position.x, vehicle.position.y, radius, vehicleColor);

    // Draw vehicle direction indicator
    if (vehicle.state == VehicleState::MOVING && !vehicle.currentPath.empty()) {
        Point direction = (vehicle.targetPosition - vehicle.position).normalize();
        Point arrowEnd = vehicle.position + direction * (radius * 1.5f);

        DrawLineEx(Vector2{vehicle.position.x, vehicle.position.y},
                   Vector2{arrowEnd.x, arrowEnd.y},
                   2.0f / camera.zoom, RED);
    }

    // Draw vehicle ID (always visible for debugging)
    std::string idText = std::to_string(vehicle.vehicleId);
    Point textPos = worldToScreen(vehicle.position);
    DrawText(idText.c_str(), textPos.x - 10, textPos.y - 20, 16, WHITE);

    // Draw target node ID if assigned
    if (vehicle.targetNodeId != -1) {
        std::string targetText = "T:" + std::to_string(vehicle.targetNodeId);
        DrawText(targetText.c_str(), textPos.x - 10, textPos.y + 5, 12, YELLOW);
    }
}

void Renderer::renderIntersection(const Point& intersection) {
    DrawCircle(intersection.x, intersection.y, 12.0f / camera.zoom, INTERSECTION_COLOR);
}

void Renderer::renderCoordinatePicker() {
    // Draw outer circle for better visibility
    DrawCircleLines(pickerPosition.x, pickerPosition.y, pickerRadius + 3.0f, BLACK);

    // Draw main picker circle
    DrawCircle(pickerPosition.x, pickerPosition.y, pickerRadius, PICKER_COLOR);

    // Draw crosshair for precise positioning
    float crossSize = pickerRadius * 0.7f;
    DrawLineEx({pickerPosition.x - crossSize, pickerPosition.y},
               {pickerPosition.x + crossSize, pickerPosition.y},
               2.0f, BLACK);
    DrawLineEx({pickerPosition.x, pickerPosition.y - crossSize},
               {pickerPosition.x, pickerPosition.y + crossSize},
               2.0f, BLACK);
}

void Renderer::updateCoordinatePicker() {
    Vector2 mousePos = GetMousePosition();
    Point worldMousePos = screenToWorld(Point(mousePos.x, mousePos.y));

    // Check if mouse is over picker
    float distance = worldMousePos.distanceTo(pickerPosition);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (distance <= pickerRadius) {
            isDraggingPicker = true;
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        isDraggingPicker = false;
    }

    // Update picker position while dragging
    if (isDraggingPicker) {
        pickerPosition = worldMousePos;
    }
}

void Renderer::renderVehicleRoute(const Auto& vehicle, const VehicleController& vehicleController, const PathSystem& pathSystem) {
    if (vehicle.currentPath.empty() || vehicle.targetNodeId == -1) return;

    Color routeColor = Color{255, 255, 0, 128}; // Semi-transparent yellow
    float routeThickness = 4.0f / camera.zoom;
    if (routeThickness < 2.0f) routeThickness = 2.0f;

    // Draw current planned path
    for (size_t i = vehicle.currentSegmentIndex; i < vehicle.currentPath.size(); i++) {
        int segmentId = vehicle.currentPath[i];
        const PathSegment* segment = pathSystem.getSegment(segmentId);

        if (segment) {
            const PathNode* startNode = pathSystem.getNode(segment->startNodeId);
            const PathNode* endNode = pathSystem.getNode(segment->endNodeId);

            if (startNode && endNode) {
                Vector2 start = {startNode->position.x, startNode->position.y};
                Vector2 end = {endNode->position.x, endNode->position.y};
                DrawLineEx(start, end, routeThickness, routeColor);
            }
        }
    }

    // Draw target node indicator
    if (vehicle.targetNodeId != -1) {
        const PathNode* targetNode = pathSystem.getNode(vehicle.targetNodeId);
        if (targetNode) {
            DrawCircle(targetNode->position.x, targetNode->position.y, 8.0f, YELLOW);
            DrawCircle(targetNode->position.x, targetNode->position.y, 6.0f, RED);
        }
    }
}

void Renderer::drawPathNetwork(const PathSystem& pathSystem) {
    // Draw segments
    for (const auto& segment : pathSystem.getSegments()) {
        const PathNode* startNode = pathSystem.getNode(segment.startNodeId);
        const PathNode* endNode = pathSystem.getNode(segment.endNodeId);

        if (startNode && endNode) {
            Vector2 start = {startNode->position.x + offset.x, startNode->position.y + offset.y};
            Vector2 end = {endNode->position.x + offset.x, endNode->position.y + offset.y};

            Color segmentColor = segment.isOccupied ? RED : DARKGRAY;
            DrawLineV(start, end, segmentColor);
        }
    }

    // Draw nodes
    for (const auto& node : pathSystem.getNodes()) {
        Vector2 nodePos = {node.position.x + offset.x, node.position.y + offset.y};
        DrawCircle((int)nodePos.x, (int)nodePos.y, (int)(8.0f * scale), BLUE);
        DrawText(std::to_string(node.nodeId).c_str(), 
                nodePos.x - 5, nodePos.y - 5, 12, WHITE);
    }
}

void Renderer::drawVehicleRoutes(const VehicleController& vehicleController, const PathSystem& pathSystem) {
    // Draw planned routes for each vehicle
    for (const auto& vehicle : vehicleController.getVehicles()) {
        if (vehicle.currentPath.empty()) continue;

        // Different colors for different vehicles
        Color routeColors[] = {GREEN, YELLOW, ORANGE, PURPLE};
        Color routeColor = routeColors[vehicle.vehicleId % 4];

        // Draw route segments
        for (size_t i = 0; i < vehicle.currentPath.size(); i++) {
            const PathSegment* segment = pathSystem.getSegment(vehicle.currentPath[i]);
            if (!segment) continue;

            const PathNode* startNode = pathSystem.getNode(segment->startNodeId);
            const PathNode* endNode = pathSystem.getNode(segment->endNodeId);

            if (startNode && endNode) {
                Vector2 start = {startNode->position.x + offset.x, startNode->position.y + offset.y};
                Vector2 end = {endNode->position.x + offset.x, endNode->position.y + offset.y};

                // Highlight current segment
                float thickness = (i == vehicle.currentSegmentIndex) ? 6.0f : 4.0f;
                DrawLineV(start, end, routeColor);
            }
        }

        // Draw target node
        if (vehicle.targetNodeId != -1) {
            const PathNode* targetNode = pathSystem.getNode(vehicle.targetNodeId);
            if (targetNode) {
                Vector2 targetPos = {targetNode->position.x + offset.x, targetNode->position.y + offset.y};
                DrawCircle((int)targetPos.x, (int)targetPos.y, (int)(12.0f * scale), routeColor);
                DrawCircleLines((int)targetPos.x, (int)targetPos.y, (int)(12.0f * scale), BLACK);
            }
        }
    }
}