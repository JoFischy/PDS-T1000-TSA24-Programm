#include "simulation.h"
#include "raylib.h"
#include <random>
#include <iostream>

Simulation::Simulation() 
    : segmentManager(nullptr), vehicleController(nullptr), renderer(nullptr),
      isRunning(false), isPaused(false), isInitialized(false), spawnTimer(0.0f),
      vehicleSpawnRate(DEFAULT_SPAWN_RATE), defaultVehicleSpeed(DEFAULT_VEHICLE_SPEED) {}

Simulation::~Simulation() {
    cleanup();
}

bool Simulation::initialize(const char* factoryImagePath) {
    // Initialize renderer
    renderer = new Renderer();
    if (!renderer->initialize(WINDOW_WIDTH, WINDOW_HEIGHT, "Factory Vehicle Simulation")) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return false;
    }

    // Load factory image
    if (!renderer->loadBackgroundImage(factoryImagePath)) {
        std::cerr << "Failed to load factory image: " << factoryImagePath << std::endl;
        return false;
    }

    // Create factory path system with predefined nodes
    createFactoryPathSystem();

    std::cout << "Created factory path system with " << pathSystem.getNodeCount() 
              << " nodes and " << pathSystem.getSegmentCount() << " segments" << std::endl;

    // Initialize traffic management
    segmentManager = new SegmentManager(&pathSystem);
    vehicleController = new VehicleController(&pathSystem, segmentManager);

    // Spawn 4 initial vehicles
    vehicleController->spawnInitialVehicles();

    isInitialized = true;
    isRunning = true;

    std::cout << "Simulation initialized successfully!" << std::endl;
    std::cout << "Detected " << pathSystem.getNodeCount() << " nodes and " 
              << pathSystem.getSegmentCount() << " segments" << std::endl;

    return true;
}

void Simulation::cleanup() {
    delete vehicleController;
    delete segmentManager;
    delete renderer;

    vehicleController = nullptr;
    segmentManager = nullptr;
    renderer = nullptr;

    isInitialized = false;
    isRunning = false;
}

void Simulation::run() {
    if (!isInitialized) return;

    while (isRunning && !renderer->shouldClose()) {
        float deltaTime = GetFrameTime();

        handleInput();

        if (!isPaused) {
            if (!update(deltaTime)) {
                isRunning = false;
                break;
            }
        }

        render();
    }
}

bool Simulation::update(float deltaTime) {
    // Update vehicles (but don't spawn new ones automatically)
    vehicleController->updateVehicles(deltaTime);

    return true;
}

void Simulation::render() {
    renderer->beginFrame();

    // Render background image
    renderer->renderBackground();

    // Render path system
    renderer->renderPathSystem(pathSystem);

    // Render vehicles
    renderer->renderVehicles(*vehicleController);

    // Render debug information
    renderer->renderDebugInfo(pathDetector, pathSystem);

    renderer->endFrame();
}

void Simulation::spawnRandomVehicle() {
    if (pathSystem.getNodeCount() == 0) return;

    // Select random start node
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, pathSystem.getNodeCount() - 1);

    int randomNodeIndex = nodeDist(gen);
    const auto& nodes = pathSystem.getNodes();
    if (randomNodeIndex < nodes.size()) {
        spawnVehicleAtNode(nodes[randomNodeIndex].nodeId);
    }
}

void Simulation::spawnVehicleAtNode(int nodeId) {
    const PathNode* node = pathSystem.getNode(nodeId);
    if (!node) return;

    int vehicleId = vehicleController->addVehicle(node->position);

    // Assign random target
    assignRandomTarget(vehicleId);
}

void Simulation::handleInput() {
    // Update camera controls
    renderer->updateCamera();

    // Pause/resume
    if (IsKeyPressed(KEY_SPACE)) {
        togglePause();
    }

    // Manual vehicle spawning
    if (IsKeyPressed(KEY_V)) {
        spawnRandomVehicle();
    }

    // Vereinfachte Fahrzeug- und Zielknotenauswahl
    static int selectedVehicle = -1;

    // Fahrzeug auswählen (1-4)
    if (IsKeyPressed(KEY_ONE)) {
        selectedVehicle = 0;
        std::cout << "Vehicle 1 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_TWO)) {
        selectedVehicle = 1;
        std::cout << "Vehicle 2 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_THREE)) {
        selectedVehicle = 2;
        std::cout << "Vehicle 3 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_FOUR)) {
        selectedVehicle = 3;
        std::cout << "Vehicle 4 selected" << std::endl;
    }

    // Mausklick auf Knoten für Zielauswahl
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && selectedVehicle >= 0 && selectedVehicle < vehicleController->getVehicleCount()) {
        Vector2 mousePos = GetMousePosition();
        Vector2 worldPos = renderer->screenToWorld(mousePos);

        // Finde nächsten Knoten zum Mausklick
        int nearestNodeId = pathSystem.findNearestNode(Point(worldPos.x, worldPos.y));
        if (nearestNodeId != -1) {
            const auto& vehicles = vehicleController->getVehicles();
            if (selectedVehicle < vehicles.size()) {
                int vehicleId = vehicles[selectedVehicle].vehicleId;
                vehicleController->setVehicleTargetNode(vehicleId, nearestNodeId);
                std::cout << "=== TARGET ASSIGNMENT ===" << std::endl;
                std::cout << "Vehicle " << (selectedVehicle + 1) << " (ID: " << vehicleId << ") target set to node " << nearestNodeId << std::endl;
                std::cout << "Current vehicle position: (" << vehicles[selectedVehicle].position.x << ", " << vehicles[selectedVehicle].position.y << ")" << std::endl;
                std::cout << "Vehicle current node: " << vehicles[selectedVehicle].currentNodeId << std::endl;
                std::cout << "Vehicle state: " << (int)vehicles[selectedVehicle].state << std::endl;
                std::cout << "=========================" << std::endl;
            }
        }
    }

    // Nummerntasten für direkte Knotenauswahl (falls gewünscht)
    if (selectedVehicle >= 0 && selectedVehicle < vehicleController->getVehicleCount()) {
        const auto& vehicles = vehicleController->getVehicles();
        if (selectedVehicle < vehicles.size()) {
            int vehicleId = vehicles[selectedVehicle].vehicleId;

            // Nummerntasten 0-9 für Knoten 1-10
            for (int i = 0; i <= 9; i++) {
                int key = KEY_ZERO + i;
                if (IsKeyPressed(key)) {
                    int targetNode = (i == 0) ? 10 : i; // 0 = Knoten 10, 1-9 = Knoten 1-9
                    if (targetNode <= 13) { // Nur gültige Knoten
                        vehicleController->setVehicleTargetNode(vehicleId, targetNode);
                        std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node " << targetNode << std::endl;
                    }
                    break;
                }
            }

            // Q, W, E für Knoten 11, 12, 13
            if (IsKeyPressed(KEY_Q)) {
                vehicleController->setVehicleTargetNode(vehicleId, 11);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node 11" << std::endl;
            }
            if (IsKeyPressed(KEY_W)) {
                vehicleController->setVehicleTargetNode(vehicleId, 12);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node 12" << std::endl;
            }
            if (IsKeyPressed(KEY_E)) {
                vehicleController->setVehicleTargetNode(vehicleId, 13);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node 13" << std::endl;
            }
        }
    }

    // Alle Fahrzeuge neue zufällige Ziele zuweisen
    if (IsKeyPressed(KEY_R)) {
        vehicleController->assignRandomTargetsToAllVehicles();
        std::cout << "Assigned new random targets to all vehicles" << std::endl;
    }

    // Speed controls
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
        vehicleSpawnRate += 0.5f;
        if (vehicleSpawnRate > 10.0f) vehicleSpawnRate = 10.0f;
    }

    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
        vehicleSpawnRate -= 0.5f;
        if (vehicleSpawnRate < 0.1f) vehicleSpawnRate = 0.1f;
    }

    // Exit
    if (IsKeyPressed(KEY_ESCAPE)) {
        isRunning = false;
    }
}

void Simulation::spawnVehicles(float deltaTime) {
    spawnTimer += deltaTime;

    float spawnInterval = 1.0f / vehicleSpawnRate;

    while (spawnTimer >= spawnInterval) {
        spawnRandomVehicle();
        spawnTimer -= spawnInterval;
    }
}

void Simulation::assignRandomTarget(int vehicleId) {
    if (pathSystem.getNodeCount() < 2) return;

    const Auto* vehicle = vehicleController->getVehicle(vehicleId);
    if (!vehicle || vehicle->currentNodeId == -1) return;

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> nodeDist(0, pathSystem.getNodeCount() - 1);

    // Find a different target node
    int targetNodeId;
    do {
        int randomIndex = nodeDist(gen);
        const auto& nodes = pathSystem.getNodes();
        if (randomIndex < nodes.size()) {
            targetNodeId = nodes[randomIndex].nodeId;
        }
    } while (targetNodeId == vehicle->currentNodeId);

    vehicleController->setVehicleTarget(vehicleId, targetNodeId);
}

void Simulation::createTestPathSystem() {
    // Create a simple rectangular path for testing
    // This creates a path system with nodes in a rectangle shape

    // Clear any existing data
    pathSystem = PathSystem();

    // Add nodes in a rectangular pattern centered on screen (1200x800 window)
    // Use smaller coordinates that are definitely visible
    // Top-left corner
    int node1 = pathSystem.addNode(300, 200);
    // Top-right corner  
    int node2 = pathSystem.addNode(900, 200);
    // Bottom-right corner
    int node3 = pathSystem.addNode(900, 600);
    // Bottom-left corner
    int node4 = pathSystem.addNode(300, 600);

    // Add some intermediate nodes for more complex paths
    int node5 = pathSystem.addNode(600, 200); // Top middle
    int node6 = pathSystem.addNode(900, 400); // Right middle
    int node7 = pathSystem.addNode(600, 600); // Bottom middle
    int node8 = pathSystem.addNode(300, 400); // Left middle

    // Add center intersection
    int centerNode = pathSystem.addNode(600, 400);

    // Connect nodes to form paths
    // Outer rectangle
    pathSystem.addSegment(node1, node5); // Top-left to top-middle
    pathSystem.addSegment(node5, node2); // Top-middle to top-right
    pathSystem.addSegment(node2, node6); // Top-right to right-middle
    pathSystem.addSegment(node6, node3); // Right-middle to bottom-right
    pathSystem.addSegment(node3, node7); // Bottom-right to bottom-middle
    pathSystem.addSegment(node7, node4); // Bottom-middle to bottom-left
    pathSystem.addSegment(node4, node8); // Bottom-left to left-middle
    pathSystem.addSegment(node8, node1); // Left-middle to top-left

    // Connect to center (creates crossroads)
    pathSystem.addSegment(node5, centerNode); // Top to center
    pathSystem.addSegment(centerNode, node6); // Center to right
    pathSystem.addSegment(centerNode, node7); // Center to bottom
    pathSystem.addSegment(node8, centerNode); // Left to center

    std::cout << "Created test path system with " << pathSystem.getNodeCount() 
              << " nodes and " << pathSystem.getSegmentCount() << " segments" << std::endl;

    // Center camera on the test path system (center is at 600, 400)
    renderer->setCameraTarget(600, 400);
}

void Simulation::createFactoryPathSystem() {
    // Clear any existing data
    pathSystem = PathSystem();

    // Create the 13 factory nodes with exact coordinates
    int node1 = pathSystem.addNode(70, 65);       // Node 1
    int node2 = pathSystem.addNode(640, 65);      // Node 2  
    int node3 = pathSystem.addNode(985, 65);      // Node 3
    int node4 = pathSystem.addNode(1860, 65);     // Node 4
    int node5 = pathSystem.addNode(70, 470);      // Node 5
    int node6 = pathSystem.addNode(640, 470);     // Node 6
    int node7 = pathSystem.addNode(985, 320);     // Node 7
    int node8 = pathSystem.addNode(1860, 320);    // Node 8
    int node9 = pathSystem.addNode(985, 750);     // Node 9
    int node10 = pathSystem.addNode(1860, 750);   // Node 10
    int node11 = pathSystem.addNode(70, 1135);    // Node 11
    int node12 = pathSystem.addNode(985, 1135);   // Node 12
    int node13 = pathSystem.addNode(1860, 1135);  // Node 13

    // Connect nodes according to specification
    // Node 1 connections: 2, 5
    pathSystem.addSegment(node1, node2);
    pathSystem.addSegment(node1, node5);

    // Node 2 connections: 1, 3, 6 (1 already connected above)
    pathSystem.addSegment(node2, node3);
    pathSystem.addSegment(node2, node6);

    // Node 3 connections: 2, 4, 7 (2 already connected above)
    pathSystem.addSegment(node3, node4);
    pathSystem.addSegment(node3, node7);

    // Node 4 connections: 3, 8 (3 already connected above)
    pathSystem.addSegment(node4, node8);

    // Node 5 connections: 1, 6, 11 (1 already connected above)
    pathSystem.addSegment(node5, node6);
    pathSystem.addSegment(node5, node11);

    // Node 6 connections: 2, 5 (already connected above)

    // Node 7 connections: 3, 8, 9 (3 already connected above)
    pathSystem.addSegment(node7, node8);
    pathSystem.addSegment(node7, node9);

    // Node 8 connections: 4, 7, 10 (4, 7 already connected above)
    pathSystem.addSegment(node8, node10);

    // Node 9 connections: 7, 10, 12 (7 already connected above)
    pathSystem.addSegment(node9, node10);
    pathSystem.addSegment(node9, node12);

    // Node 10 connections: 8, 9, 13 (8, 9 already connected above)
    pathSystem.addSegment(node10, node13);

    // Node 11 connections: 5, 12 (5 already connected above)
    pathSystem.addSegment(node11, node12);

    // Node 12 connections: 9, 11, 13 (9, 11 already connected above)
    pathSystem.addSegment(node12, node13);

    // Node 13 connections: 10, 12 (already connected above)

    std::cout << "Created factory path system with predefined coordinates:" << std::endl;
    std::cout << "13 nodes and " << pathSystem.getSegmentCount() << " segments" << std::endl;

    // Center camera on the factory layout (roughly in the middle)
    renderer->setCameraTarget(965, 600);
}