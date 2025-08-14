#pragma once
#include "path_system.h"
#include "vehicle_controller.h"
#include "path_detector.h"
#include "raylib.h"

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();

    // Initialization
    bool initialize(int windowWidth, int windowHeight, const char* title);
    void cleanup();

    // Background image
    bool loadBackgroundImage(const char* imagePath);

    // Main rendering
    void beginFrame();
    void endFrame();
    void renderBackground();
    void renderPathSystem(const PathSystem& pathSystem);
    void renderVehicles(const VehicleController& vehicleController);
    void renderDebugInfo(const PathDetector& pathDetector, const PathSystem& pathSystem);
    void renderUI();

    // Camera controls
    void updateCamera();
    void setCameraTarget(float x, float y);
    void resetCamera();
    void zoomIn();
    void zoomOut();
    void fitToView();  // Neue Methode: Passt die Ansicht an das gesamte Bild an
    void toggleFullscreen();  // Neue Methode: Vollbildmodus umschalten
    Vector2 screenToWorld(Vector2 screenPos) const;

    // Settings
    void setShowNodes(bool show) { showNodes = show; }
    void setShowSegments(bool show) { showSegments = show; }
    void setShowIntersections(bool show) { showIntersections = show; }
    void setShowVehicleIds(bool show) { showVehicleIds = show; }
    void setShowDebugInfo(bool show) { showDebugInfo = show; }

    bool getShowNodes() const { return showNodes; }
    bool getShowSegments() const { return showSegments; }
    bool getShowIntersections() const { return showIntersections; }
    bool getShowVehicleIds() const { return showVehicleIds; }
    bool getShowDebugInfo() const { return showDebugInfo; }

    // Coordinate picker
    void enableCoordinatePicker(bool enable) { coordinatePickerEnabled = enable; }
    bool isCoordinatePickerEnabled() const { return coordinatePickerEnabled; }
    Point getPickerCoordinates() const { return pickerPosition; }

    // Utility
    Point screenToWorld(const Point& screenPos) const;
    Point worldToScreen(const Point& worldPos) const;
    bool shouldClose() const;

private:
    void renderNode(const PathNode& node);
    void renderSegment(const PathSegment& segment, const PathSystem& pathSystem);
    void renderVehicle(const Auto& vehicle);
    void renderVehicleRoute(const Auto& vehicle, const VehicleController& vehicleController, const PathSystem& pathSystem);
    void renderIntersection(const Point& intersection);
    void renderCoordinatePicker();
    void updateCoordinatePicker();
    void drawPathNetwork(const PathSystem& pathSystem);
    void drawVehicleRoutes(const VehicleController& vehicleController, const PathSystem& pathSystem);

    // Graphics resources
    Texture2D backgroundTexture;
    Camera2D camera;
    bool hasBackgroundImage;
    Vector2 offset;
    float scale;

    // Rendering settings
    bool showNodes;
    bool showSegments;
    bool showIntersections;
    bool showVehicleIds;
    bool showDebugInfo;

    // Coordinate picker
    bool coordinatePickerEnabled;
    Point pickerPosition;
    bool isDraggingPicker;
    float pickerRadius;

    // Window properties
    int windowWidth;
    int windowHeight;
    bool isInitialized;

    // Colors
    static const Color NODE_COLOR;
    static const Color SEGMENT_COLOR;
    static const Color OCCUPIED_SEGMENT_COLOR;
    static const Color VEHICLE_COLOR;
    static const Color INTERSECTION_COLOR;
    static const Color UI_BACKGROUND_COLOR;
    static const Color UI_TEXT_COLOR;
    static const Color PICKER_COLOR;
    
    // Node type colors
    static const Color WAITING_NODE_COLOR;
    static const Color WAITING_NODE_BORDER_COLOR;
    static const Color T_JUNCTION_COLOR;
    static const Color T_JUNCTION_BORDER_COLOR;
    static const Color CROSSROAD_COLOR;
    static const Color CROSSROAD_BORDER_COLOR;
    static const Color CURVE_NODE_COLOR;
    static const Color CURVE_NODE_BORDER_COLOR;
    
    // Vehicle color system
    static const Color VEHICLE_COLORS[];
    static const Color WAITING_COLORS[];
    static const int NUM_VEHICLE_COLORS;
};