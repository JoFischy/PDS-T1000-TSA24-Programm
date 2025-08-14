#pragma once
#include "path_system.h"
#include "segment_manager.h"
#include "vehicle_controller.h"
#include "renderer.h"
#include "raylib.h"

class Simulation {
public:
    Simulation();
    ~Simulation();

    // Initialization
    bool initialize(const char* factoryImagePath);
    void cleanup();

    // Main loop
    void run();
    bool update(float deltaTime);
    void render();

    // Vehicle spawning
    void spawnRandomVehicle();
    void spawnVehicleAtNode(int nodeId);

    // Controls
    void pause() { isPaused = true; }
    void resume() { isPaused = false; }
    void togglePause() { isPaused = !isPaused; }
    bool getPaused() const { return isPaused; }

    // Settings
    void setSpawnRate(float rate) { vehicleSpawnRate = rate; }
    void setVehicleSpeed(float speed) { defaultVehicleSpeed = speed; }

private:
    void handleInput();
    void spawnVehicles(float deltaTime);
    void assignRandomTarget(int vehicleId);
    void createTestPathSystem();
    void createFactoryPathSystem();

    // Core systems
    PathSystem pathSystem;
    SegmentManager* segmentManager;
    VehicleController* vehicleController;
    Renderer* renderer;

    // Simulation state
    bool isRunning;
    bool isPaused;
    bool isInitialized;

    // Timing
    float spawnTimer;
    float vehicleSpawnRate;
    float defaultVehicleSpeed;

    // Constants
    static constexpr int WINDOW_WIDTH = 1200;
    static constexpr int WINDOW_HEIGHT = 800;
    static constexpr float DEFAULT_SPAWN_RATE = 2.0f; // vehicles per second
    static constexpr float DEFAULT_VEHICLE_SPEED = 50.0f;
};