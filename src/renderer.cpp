#include "renderer.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

Renderer::Renderer(int width, int height) 
    : hasBackgroundImage(false), showNodes(true), showSegments(true), 
      showIntersections(true), showVehicleIds(false), showDebugInfo(false),
      windowWidth(width), windowHeight(height), isInitialized(false),
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
    // Ermittle Monitor-Auflösung vor der Fenster-Erstellung
    InitWindow(windowWidth, windowHeight, title);
    
    // Sofortiger Wechsel zu Vollbild nach Fenster-Erstellung
    ToggleFullscreen();
    SetTargetFPS(60);

    // Nach Vollbild-Wechsel: Hole tatsächliche Bildschirm-Dimensionen
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();
    
    // Aktualisiere interne Dimensionen
    this->windowWidth = screenWidth;
    this->windowHeight = screenHeight;

    // Initialize camera für Vollbild - starte in der Bildmitte
    camera.target = {960, 600}; // Mitte des 1920x1200 Hintergrundbildes
    camera.offset = {screenWidth / 2.0f, screenHeight / 2.0f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f; // Starte mit normalem Zoom

    isInitialized = true;
    std::cout << "Fullscreen mode activated: " << screenWidth << "x" << screenHeight << std::endl;
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
    // Draw UI background - increased width for node type legend
    DrawRectangle(10, 10, 320, 450, UI_BACKGROUND_COLOR);

    // Camera info
    char cameraInfo[128];
    snprintf(cameraInfo, sizeof(cameraInfo), "Camera: (%.0f, %.0f) Zoom: %.2f", 
             camera.target.x, camera.target.y, camera.zoom);
    DrawText(cameraInfo, 15, 15, 14, LIGHTGRAY);

    // Draw text information
    int yOffset = 35;
    DrawText("Factory Vehicle Simulation", 15, yOffset, 16, UI_TEXT_COLOR);
    yOffset += 25;

    // Camera controls
    DrawText("Camera Controls:", 15, yOffset, 14, YELLOW);
    yOffset += 20;
    DrawText("WASD - Move camera", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("Mouse wheel - Zoom", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("F - Fit to view", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("R - Reset camera", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("F11 - Toggle fullscreen", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 20;

    DrawText("Vehicle Selection:", 15, yOffset, 14, YELLOW);
    yOffset += 20;
    DrawText("F1,F2,F3,F4 - Select vehicle", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("Click node - Set target", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("R - Random targets for all", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("T - Toggle auto-assign", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 20;

    DrawText("Camera Controls:", 15, yOffset, 14, UI_TEXT_COLOR);
    yOffset += 20;
    DrawText("Mouse wheel - Zoom", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 20;

    DrawText("Other Controls:", 15, yOffset, 14, UI_TEXT_COLOR);
    yOffset += 20;
    DrawText("Space - Pause/Resume", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("V - Spawn new vehicle", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("ESC - Exit", 15, yOffset, 12, UI_TEXT_COLOR);
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

    DrawText("Node Types:", 15, yOffset, 14, YELLOW);
    yOffset += 20;

    // Draw node type legend with colored circles
    DrawCircle(25, yOffset + 6, 6, NODE_COLOR);
    DrawText("Blue - Regular nodes (0-1 connections)", 40, yOffset, 10, UI_TEXT_COLOR);
    yOffset += 15;

    DrawCircle(25, yOffset + 6, 6, CURVE_NODE_COLOR);
    DrawText("Green - Curve nodes (2 connections)", 40, yOffset, 10, UI_TEXT_COLOR);
    yOffset += 15;

    DrawCircle(25, yOffset + 6, 6, T_JUNCTION_COLOR);
    DrawText("Yellow - T-junctions (3+ connections)", 40, yOffset, 10, UI_TEXT_COLOR);
    yOffset += 15;

    DrawCircle(25, yOffset + 6, 6, WAITING_NODE_COLOR);
    DrawText("Orange - Waiting nodes", 40, yOffset, 10, UI_TEXT_COLOR);
    yOffset += 20;

    DrawText("Vehicle Colors:", 15, yOffset, 14, YELLOW);
    yOffset += 20;
    DrawText("Each vehicle has unique color", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;
    DrawText("Bright - Moving/Idle", 15, yOffset, 12, {200, 200, 200, 255});
    yOffset += 15;
    DrawText("Dimmed - Waiting", 15, yOffset, 12, {150, 150, 150, 255});
    yOffset += 15;
    DrawText("Routes match vehicle colors", 15, yOffset, 12, UI_TEXT_COLOR);
    yOffset += 15;

    DrawText("Segments: Gray=free, Purple=blocked", 15, yOffset, 10, UI_TEXT_COLOR);
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

    // WASD camera movement
    float moveSpeed = 500.0f / camera.zoom; // Erhöhte Bewegungsgeschwindigkeit
    bool moved = false;
    if (IsKeyDown(KEY_W)) { camera.target.y -= moveSpeed * GetFrameTime(); moved = true; }
    if (IsKeyDown(KEY_S)) { camera.target.y += moveSpeed * GetFrameTime(); moved = true; }
    if (IsKeyDown(KEY_A)) { camera.target.x -= moveSpeed * GetFrameTime(); moved = true; }
    if (IsKeyDown(KEY_D)) { camera.target.x += moveSpeed * GetFrameTime(); moved = true; }
    
    static float debugTimer = 0;
    debugTimer += GetFrameTime();
    if (moved && debugTimer > 0.5f) { // Häufigere Debug-Ausgabe
        std::cout << "Camera moved to: (" << camera.target.x << ", " << camera.target.y << ") zoom=" << camera.zoom << std::endl;
        debugTimer = 0;
    }

    // Zoom with mouse wheel
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        camera.zoom += wheel * 0.1f * camera.zoom;
        if (camera.zoom < 0.1f) camera.zoom = 0.1f;
        if (camera.zoom > 5.0f) camera.zoom = 5.0f;
    }

    // Toggle coordinate picker
    if (IsKeyPressed(KEY_P)) {
        coordinatePickerEnabled = !coordinatePickerEnabled;
    }

    // Toggle display options with number keys
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

void Renderer::fitToView() {
    if (hasBackgroundImage) {
        // Berechne den Zoom-Faktor, um das gesamte Bild anzuzeigen
        // Verwende aktuelle Bildschirmauflösung
        int currentScreenWidth = GetScreenWidth();
        int currentScreenHeight = GetScreenHeight();
        
        float zoomX = (float)currentScreenWidth / backgroundTexture.width;
        float zoomY = (float)currentScreenHeight / backgroundTexture.height;
        camera.zoom = (zoomX < zoomY) ? zoomX : zoomY;
        
        // Zentriere das Bild
        camera.target = {backgroundTexture.width / 2.0f, backgroundTexture.height / 2.0f};
        
        std::cout << "Fit to view: screen=" << currentScreenWidth << "x" << currentScreenHeight 
                  << ", zoom=" << camera.zoom << ", target=(" 
                  << camera.target.x << "," << camera.target.y << ")" << std::endl;
    }
}

void Renderer::toggleFullscreen() {
    if (IsWindowFullscreen()) {
        // Verlasse Vollbildmodus
        ToggleFullscreen();
        SetWindowSize(windowWidth, windowHeight);
        camera.offset = {windowWidth / 2.0f, windowHeight / 2.0f};
        std::cout << "Exited fullscreen mode (" << windowWidth << "x" << windowHeight << ")" << std::endl;
    } else {
        // Wechsle zu Vollbildmodus
        int monitor = GetCurrentMonitor();
        int monitorWidth = GetMonitorWidth(monitor);
        int monitorHeight = GetMonitorHeight(monitor);
        
        ToggleFullscreen();
        camera.offset = {monitorWidth / 2.0f, monitorHeight / 2.0f};
        std::cout << "Entered fullscreen mode (" << monitorWidth << "x" << monitorHeight << ")" << std::endl;
    }
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
    float nodeRadius = 18.0f / camera.zoom;
    if (nodeRadius < 12.0f) nodeRadius = 12.0f; // Minimum size

    // Determine node type and color
    Color nodeColor = NODE_COLOR;
    Color borderColor = BLACK;
    bool drawSpecialIndicator = false;

    // Check if it's a waiting node
    if (node.isWaitingNode) {
        nodeColor = WAITING_NODE_COLOR;
        borderColor = WAITING_NODE_BORDER_COLOR;
        drawSpecialIndicator = true;
    } else {
        // Check connection count to determine node type
        int connectionCount = node.connectedSegments.size();

        // Force Node 1 to be curve node regardless of connection count
        if (node.nodeId == 1) {
            nodeColor = CURVE_NODE_COLOR;
            borderColor = CURVE_NODE_BORDER_COLOR;
        } else if (connectionCount >= 3) {
            // All junctions with 3+ connections are T-junctions (no crossroads in this system)
            nodeColor = T_JUNCTION_COLOR;
            borderColor = T_JUNCTION_BORDER_COLOR;
            drawSpecialIndicator = true;
        } else if (connectionCount == 2) {
            // Curve node
            nodeColor = CURVE_NODE_COLOR;
            borderColor = CURVE_NODE_BORDER_COLOR;
        }
        // Regular nodes (1 or 0 connections) keep default colors
    }

    // Draw border
    DrawCircle(node.position.x, node.position.y, nodeRadius + 3, borderColor);
    // Draw main node
    DrawCircle(node.position.x, node.position.y, nodeRadius, nodeColor);

    // Draw special indicator for junctions and waiting nodes
    if (drawSpecialIndicator) {
        if (node.isWaitingNode) {
            // Draw diamond shape for waiting nodes
            float diamondSize = nodeRadius * 0.6f;
            Vector2 center = {node.position.x, node.position.y};
            Vector2 points[4] = {
                {center.x, center.y - diamondSize},  // Top
                {center.x + diamondSize, center.y},  // Right
                {center.x, center.y + diamondSize},  // Bottom
                {center.x - diamondSize, center.y}   // Left
            };

            for (int i = 0; i < 4; i++) {
                Vector2 start = points[i];
                Vector2 end = points[(i + 1) % 4];
                DrawLineEx(start, end, 2.0f, WHITE);
            }
        } else {
            // Draw cross for junctions
            float crossSize = nodeRadius * 0.7f;
            DrawLineEx({node.position.x - crossSize, node.position.y},
                      {node.position.x + crossSize, node.position.y},
                      3.0f, WHITE);
            DrawLineEx({node.position.x, node.position.y - crossSize},
                      {node.position.x, node.position.y + crossSize},
                      3.0f, WHITE);
        }
    }

    // Draw node ID - keep text in screen space to maintain consistent size and positioning
    std::string nodeText = std::to_string(node.nodeId);
    Point screenPos = worldToScreen(node.position);

    // Fixed text size independent of zoom
    int fontSize = 14;
    int textWidth = MeasureText(nodeText.c_str(), fontSize);

    // Center text on node in screen coordinates
    DrawText(nodeText.c_str(), 
             screenPos.x - textWidth/2, 
             screenPos.y - fontSize/2, 
             fontSize, WHITE);
}

void Renderer::renderSegment(const PathSegment& segment, const PathSystem& pathSystem) {
    const PathNode* startNode = pathSystem.getNode(segment.startNodeId);
    const PathNode* endNode = pathSystem.getNode(segment.endNodeId);

    if (!startNode || !endNode) return;

    Color segmentColor = segment.isOccupied ? OCCUPIED_SEGMENT_COLOR : SEGMENT_COLOR;
    float thickness = 8.0f / camera.zoom;
    if (thickness < 4.0f) thickness = 4.0f; // Minimum thickness

    // Draw segment background (darker)
    DrawLineEx({startNode->position.x, startNode->position.y},
               {endNode->position.x, endNode->position.y},
               thickness + 2, {50, 50, 50, 255});

    // Draw main segment
    DrawLineEx({startNode->position.x, startNode->position.y},
               {endNode->position.x, endNode->position.y},
               thickness, segmentColor);
}

void Renderer::renderVehicle(const Auto& vehicle) {
    // Get vehicle's unique color
    Color baseColor = VEHICLE_COLORS[vehicle.vehicleId % NUM_VEHICLE_COLORS];
    Color vehicleColor;

    // Determine color based on state
    switch (vehicle.state) {
        case VehicleState::WAITING:
            vehicleColor = WAITING_COLORS[vehicle.vehicleId % NUM_VEHICLE_COLORS];
            break;
        case VehicleState::ARRIVED:
            // Slightly dimmed version for arrived vehicles
            vehicleColor = {
                (unsigned char)(baseColor.r * 0.7f),
                (unsigned char)(baseColor.g * 0.7f),
                (unsigned char)(baseColor.b * 0.7f),
                255
            };
            break;
        case VehicleState::IDLE:
        case VehicleState::MOVING:
        default:
            vehicleColor = baseColor;
            break;
    }

    float vehicleSize = 12.0f / camera.zoom;
    if (vehicleSize < 8.0f) vehicleSize = 8.0f;
    if (vehicleSize > 20.0f) vehicleSize = 20.0f;

    Vector2 pos = {vehicle.position.x, vehicle.position.y};
    DrawCircleV(pos, vehicleSize, vehicleColor);

    // Draw outline
    DrawCircleLinesV(pos, vehicleSize, BLACK);

    // Draw vehicle ID number
    float textSize = 12.0f / camera.zoom;
    if (textSize < 8.0f) textSize = 8.0f;
    if (textSize > 16.0f) textSize = 16.0f;

    std::string idText = std::to_string(vehicle.vehicleId + 1); // 1-based numbering
    Vector2 textPos = {pos.x - textSize/4, pos.y - textSize/2};
    DrawTextEx(GetFontDefault(), idText.c_str(), textPos, textSize, 1, WHITE);

    // Draw waiting indicator if waiting at safety stop
    if (vehicle.state == VehicleState::WAITING) {
        float indicatorSize = vehicleSize * 1.8f;
        DrawCircleLinesV(pos, indicatorSize, vehicleColor);
        DrawCircleLinesV(pos, indicatorSize + 3, {255, 255, 255, 150});
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

    // Use same color as vehicle but with transparency for route
    Color baseColor = VEHICLE_COLORS[vehicle.vehicleId % NUM_VEHICLE_COLORS];
    Color routeColor = {baseColor.r, baseColor.g, baseColor.b, 160}; // Semi-transparent

    float routeThickness = 6.0f / camera.zoom;
    if (routeThickness < 3.0f) routeThickness = 3.0f;

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

    // Draw target node indicator with vehicle's color
    if (vehicle.targetNodeId != -1) {
        const PathNode* targetNode = pathSystem.getNode(vehicle.targetNodeId);
        if (targetNode) {
            DrawCircle(targetNode->position.x, targetNode->position.y, 12.0f, BLACK);
            DrawCircle(targetNode->position.x, targetNode->position.y, 10.0f, baseColor);
            DrawCircle(targetNode->position.x, targetNode->position.y, 6.0f, WHITE);
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

// Static color definitions - placed at end to avoid compilation issues
const Color Renderer::NODE_COLOR = {70, 130, 255, 255};
const Color Renderer::SEGMENT_COLOR = {150, 150, 150, 255};
const Color Renderer::OCCUPIED_SEGMENT_COLOR = {200, 50, 200, 255};
const Color Renderer::INTERSECTION_COLOR = {255, 100, 200, 255};
const Color Renderer::UI_BACKGROUND_COLOR = {20, 20, 20, 220};
const Color Renderer::UI_TEXT_COLOR = {255, 255, 255, 255};
const Color Renderer::PICKER_COLOR = {0, 255, 100, 255};

// Node type colors
const Color Renderer::WAITING_NODE_COLOR = {255, 150, 0, 255};        // Orange for waiting nodes
const Color Renderer::WAITING_NODE_BORDER_COLOR = {200, 100, 0, 255}; // Dark orange border
const Color Renderer::T_JUNCTION_COLOR = {255, 255, 0, 255};          // Yellow for T-junctions
const Color Renderer::T_JUNCTION_BORDER_COLOR = {200, 200, 0, 255};   // Dark yellow border
const Color Renderer::CROSSROAD_COLOR = {255, 0, 0, 255};             // Red for crossroads (4+ connections)
const Color Renderer::CROSSROAD_BORDER_COLOR = {200, 0, 0, 255};      // Dark red border
const Color Renderer::CURVE_NODE_COLOR = {0, 255, 150, 255};          // Green for curve nodes
const Color Renderer::CURVE_NODE_BORDER_COLOR = {0, 200, 100, 255};   // Dark green border

// Vehicle color arrays
const Color Renderer::VEHICLE_COLORS[] = {
    {255, 80, 80, 255},   // Bright Red
    {80, 255, 80, 255},   // Bright Green  
    {80, 80, 255, 255},   // Bright Blue
    {255, 255, 80, 255},  // Bright Yellow
    {255, 80, 255, 255},  // Bright Magenta
    {80, 255, 255, 255},  // Bright Cyan
    {255, 150, 80, 255},  // Bright Orange
    {150, 80, 255, 255},  // Bright Purple
    {255, 80, 150, 255},  // Bright Pink
    {150, 255, 80, 255}   // Bright Lime
};

const Color Renderer::WAITING_COLORS[] = {
    {180, 60, 60, 255},   // Dimmed Red
    {60, 180, 60, 255},   // Dimmed Green
    {60, 60, 180, 255},   // Dimmed Blue
    {180, 180, 60, 255},  // Dimmed Yellow
    {180, 60, 180, 255},  // Dimmed Magenta
    {60, 180, 180, 255},  // Dimmed Cyan
    {180, 120, 60, 255},  // Dimmed Orange
    {120, 60, 180, 255},  // Dimmed Purple
    {180, 60, 120, 255},  // Dimmed Pink
    {120, 180, 60, 255}   // Dimmed Lime
};

const int Renderer::NUM_VEHICLE_COLORS = 10;