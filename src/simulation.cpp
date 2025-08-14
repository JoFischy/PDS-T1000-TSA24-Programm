#include "simulation.h"
#include "raylib.h"
#include <random>
#include <iostream>

Simulation::Simulation() 
    : segmentManager(nullptr), vehicleController(nullptr), renderer(nullptr),
      isRunning(false), isPaused(false), isInitialized(false), spawnTimer(0.0f),
      vehicleSpawnRate(2.0f), defaultVehicleSpeed(100.0f) {}

Simulation::~Simulation() {
    cleanup();
}

bool Simulation::initialize(const char* factoryImagePath) {
    // Initialize renderer
    renderer = new Renderer(1200, 800);
    if (!renderer) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return false;
    }

    // Initialize window
    if (!renderer->initialize(1200, 800, "PDS-T1000 Factory Vehicle Simulation")) {
        std::cerr << "Failed to initialize renderer window" << std::endl;
        return false;
    }
    std::cout << "Renderer window initialized successfully!" << std::endl;

    // Load factory image
    if (!renderer->loadBackgroundImage(factoryImagePath)) {
        std::cerr << "Failed to load factory image: " << factoryImagePath << std::endl;
        return false;
    }
    std::cout << "Factory image loaded successfully!" << std::endl;

    // Create factory path system with predefined nodes
    std::cout << "Creating factory path system..." << std::endl;
    createFactoryPathSystem();
    std::cout << "Factory path system created!" << std::endl;

    std::cout << "Created factory path system with " << pathSystem.getNodeCount() 
              << " nodes and " << pathSystem.getSegmentCount() << " segments" << std::endl;

    // Initialize traffic management
    segmentManager = new SegmentManager(&pathSystem);
    vehicleController = new VehicleController(&pathSystem, segmentManager);

    // Spawn 4 initial vehicles
    vehicleController->spawnInitialVehicles();

    isInitialized = true;
    isRunning = true;

    // Automatically apply fit to view after initialization
    renderer->fitToView();
    std::cout << "Applied automatic fit to view!" << std::endl;

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

    while (isRunning && !WindowShouldClose()) {
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
    // Use renderer's frame management for proper camera transformation
    if (renderer) {
        renderer->beginFrame();  // This applies camera transformation
        
        renderer->renderBackground();
        renderer->renderPathSystem(pathSystem);
        renderer->renderVehicles(*vehicleController);
        
        renderer->endFrame();  // This renders UI and ends the frame
    }
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
        isPaused = !isPaused;
        std::cout << "Simulation " << (isPaused ? "paused" : "resumed") << std::endl;
    }

    // Camera controls
    if (IsKeyPressed(KEY_F11)) {
        renderer->toggleFullscreen();
        std::cout << "F11 pressed - Toggle fullscreen" << std::endl;
    }
    
    if (IsKeyPressed(KEY_R)) {
        renderer->resetCamera();
        std::cout << "R pressed - Camera reset" << std::endl;
    }
    
    if (IsKeyPressed(KEY_F)) {
        renderer->fitToView();
        std::cout << "F pressed - Fit to view" << std::endl;
    }

    // Manual vehicle spawning
    if (IsKeyPressed(KEY_V)) {
        spawnRandomVehicle();
    }

    // Fahrzeug- und Zielknotenauswahl mit F1-F4 um Konflikte zu vermeiden
    static int selectedVehicle = -1;

    // Fahrzeug auswählen (F1-F4)
    if (IsKeyPressed(KEY_F1)) {
        selectedVehicle = 0;
        std::cout << "Vehicle 1 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_F2)) {
        selectedVehicle = 1;
        std::cout << "Vehicle 2 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_F3)) {
        selectedVehicle = 2;
        std::cout << "Vehicle 3 selected" << std::endl;
    }
    if (IsKeyPressed(KEY_F4)) {
        selectedVehicle = 3;
        std::cout << "Vehicle 4 selected" << std::endl;
    }

    // Mausklick auf Knoten für Zielauswahl
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && selectedVehicle >= 0 && selectedVehicle < vehicleController->getVehicleCount()) {
        Vector2 mousePos = GetMousePosition();
        Point worldPos = Point(mousePos.x, mousePos.y);

        // Finde nächsten Knoten zum Mausklick (größerer Radius für bessere Auswahl)
        int nearestNodeId = pathSystem.findNearestNode(Point(worldPos.x, worldPos.y), 80.0f);
        if (nearestNodeId != -1) {
            const auto& vehicles = vehicleController->getVehicles();
            if (selectedVehicle < vehicles.size()) {
                int vehicleId = vehicles[selectedVehicle].vehicleId;
                vehicleController->setVehicleTargetNode(vehicleId, nearestNodeId);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node " << nearestNodeId << std::endl;
            }
        } else {
            std::cout << "No node found near mouse click. Try clicking closer to a node." << std::endl;
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
            if (IsKeyPressed(KEY_Y)) {
                vehicleController->setVehicleTargetNode(vehicleId, 12);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node 12" << std::endl;
            }
            if (IsKeyPressed(KEY_X)) {
                vehicleController->setVehicleTargetNode(vehicleId, 13);
                std::cout << "Vehicle " << (selectedVehicle + 1) << " target set to node 13" << std::endl;
            }
        }
    }

    // Toggle für automatische neue Ziele bei Ankunft
    static bool autoAssignTargets = false;
    if (IsKeyPressed(KEY_T)) {
        autoAssignTargets = !autoAssignTargets;
        std::cout << "Auto-assign random targets on arrival: " << (autoAssignTargets ? "ON" : "OFF") << std::endl;
    }

    // Alle Fahrzeuge neue zufällige Ziele zuweisen
    if (IsKeyPressed(KEY_R)) {
        vehicleController->assignRandomTargetsToAllVehicles();
        std::cout << "Assigned new random targets to all vehicles" << std::endl;
    }

    // Check for vehicles that arrived and need new targets
    if (autoAssignTargets) {
        for (const auto& vehicle : vehicleController->getVehicles()) {
            if (vehicle.state == VehicleState::ARRIVED && vehicle.currentNodeId == vehicle.targetNodeId) {
                vehicleController->assignNewRandomTarget(vehicle.vehicleId);
            }
        }
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

    // Add warehouse points
    int warehouse1 = pathSystem.addNode(350, 205);  // Warehouse 1
    int warehouse2 = pathSystem.addNode(505, 965);  // Warehouse 2  
    int warehouse3 = pathSystem.addNode(1410, 475); // Warehouse 3

    // Add waiting points at T-junctions (150 pixel distance in all directions)
    // T-junction at Node 2 (640, 65) - has connections: left, bottom, right
    int wait2_left = pathSystem.addWaitingNode(640 - 150, 65);     // Left wait point
    int wait2_bottom = pathSystem.addWaitingNode(640, 65 + 150);   // Bottom wait point
    // Merged waiting point between Node 2 and Node 3 (x = (640+985)/2 = 812.5)
    int wait2_3_merged = pathSystem.addWaitingNode(812, 65);       // Merged waiting point

    // T-junction at Node 3 (985, 65) - has 3 connections  
    int wait3_east = pathSystem.addWaitingNode(985 + 150, 65);     // East wait point

    // T-junction at Node 5 (70, 470) - add missing waiting points
    int wait5_top = pathSystem.addWaitingNode(70, 470 - 150);      // Top wait point
    int wait5_right = pathSystem.addWaitingNode(70 + 150, 470);    // Right wait point
    int wait5_bottom = pathSystem.addWaitingNode(70, 470 + 150);   // Bottom wait point

    // Node 6 (640, 470) - regular junction, no waiting points needed

    // T-junction at Node 7 (985, 320) - has 3 connections
    // Single waiting point between Node 3 and Node 7 (y = (65+320)/2 = 192.5)
    int wait3_7_merged = pathSystem.addWaitingNode(985, 192);      // Single waiting point between Node 3 and 7
    int wait7_east = pathSystem.addWaitingNode(985 + 150, 320);    // East wait point  
    // Merged south point between Node 7 and Node 9 (y = (320+750)/2 = 535)
    int wait7_south_merged = pathSystem.addWaitingNode(985, 535);  // Merged south wait point

    // T-junction at Node 8 (1860, 320) - has 3 connections
    int wait8_west = pathSystem.addWaitingNode(1860 - 150, 320);   // West wait point
    // No south waiting point - using shared upper waiting node from Node 10
    // Merged north-south point between Node 8 and Node 10 (y = (320+750)/2 = 535)
    int wait8_10_merged = pathSystem.addWaitingNode(1860, 535);    // Merged north-south wait point

    // T-junction at Node 9 (985, 750) - has 3 connections
    // Uses merged points from Node 7
    int wait9_east = pathSystem.addWaitingNode(985 + 150, 750);    // East wait point
    // Merged south point between Node 9 and Node 12 (y = (750+1135)/2 = 942.5)
    int wait9_south_merged = pathSystem.addWaitingNode(985, 942);  // Merged south wait point

    // Waiting point at warehouse2_access (505, 1135) 
    int wait11_left = pathSystem.addWaitingNode(505 - 150, 1135);  // Left wait point
    int wait11_right = pathSystem.addWaitingNode(505 + 150, 1135); // Right wait point

    // T-junction at Node 12 (985, 1135) - has 3 connections
    // Uses merged point from Node 9
    int wait12_east = pathSystem.addWaitingNode(985 + 150, 1135);  // East wait point
    int wait12_west = pathSystem.addWaitingNode(985 - 150, 1135);  // West wait point

    // T-junction at Node 10 (1860, 750) - has 3 connections
    // Uses merged point from Node 8 (wait8_10_merged) for north direction
    int wait10_left = pathSystem.addWaitingNode(1860 - 150, 750);  // Left wait point
    int wait10_bottom = pathSystem.addWaitingNode(1860, 750 + 150); // Bottom wait point

    // Node 13 (1860, 1135) - no waiting points (reverted)

    // Connect main nodes according to specification
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

    // Node 7 connections: 3, 6, 8, 9 (3,6 already connected above)
    pathSystem.addSegment(node7, node8);
    pathSystem.addSegment(node7, node9);

    // Node 8 connections: 4, 7, 10 (4,7 already connected above)
    pathSystem.addSegment(node8, node10);

    // Node 9 connections: 7, 10, 12 (7 already connected above)
    pathSystem.addSegment(node9, node10);
    pathSystem.addSegment(node9, node12);

    // Node 10 connections: 8, 9, 13 (8,9 already connected above)
    pathSystem.addSegment(node10, node13);

    // Node 11 connections: 5, 12 (5 already connected above)
    pathSystem.addSegment(node11, node12);

    // Node 12 connections: 9, 11, 13 (9,11 already connected above)
    pathSystem.addSegment(node12, node13);

    // Node 13 connections: 10, 12 (already connected above)

    // Connect warehouses to nearby paths
    // Warehouse 1 (350, 205) - connect to path between node1-node2
    int warehouse1_access = pathSystem.addNode(350, 65);    // Access point on main path
    pathSystem.addSegment(warehouse1_access, warehouse1);
    // Insert access point into main path
    pathSystem.addSegment(node1, warehouse1_access);
    pathSystem.addSegment(warehouse1_access, node2);

    // Warehouse 2 (505, 965) - connect to path near node9/node12
    int warehouse2_access = pathSystem.addNode(505, 1135);  // Access point on node12 level
    pathSystem.addSegment(warehouse2_access, warehouse2);
    pathSystem.addSegment(node12, warehouse2_access);

    // Warehouse 3 (1410, 475) - connect to path between node7-node8  
    int warehouse3_access = pathSystem.addNode(1410, 320);  // Access point on main path
    pathSystem.addSegment(warehouse3_access, warehouse3);
    pathSystem.addSegment(node7, warehouse3_access);
    pathSystem.addSegment(warehouse3_access, node8);

    // Connect waiting points to T-junctions
    // Node 2 waiting points 
    pathSystem.addSegment(node2, wait2_left);
    pathSystem.addSegment(node2, wait2_bottom);  
    pathSystem.addSegment(node2, wait2_3_merged);
    
    // Node 3 waiting points
    pathSystem.addSegment(node3, wait2_3_merged);  // Connect to merged point
    pathSystem.addSegment(node3, wait3_east);
    pathSystem.addSegment(node3, wait3_7_merged);  // Connect to single waiting point

    // Node 5 waiting points
    pathSystem.addSegment(node5, wait5_top);
    pathSystem.addSegment(node5, wait5_right);
    pathSystem.addSegment(node5, wait5_bottom);

    // Node 6 has no waiting points (regular junction)

    // Node 7 waiting points (using merged points)
    pathSystem.addSegment(node7, wait3_7_merged);  // Connect to single waiting point
    pathSystem.addSegment(node7, wait7_east);
    pathSystem.addSegment(node7, wait7_south_merged);
    // Also connect merged point to Node 9
    pathSystem.addSegment(node9, wait7_south_merged);

    // Node 8 waiting points
    pathSystem.addSegment(node8, wait8_west);
    pathSystem.addSegment(node8, wait8_10_merged);

    // Node 9 waiting points (using merged points)
    pathSystem.addSegment(node9, wait9_east);
    pathSystem.addSegment(node9, wait9_south_merged);
    // Also connect merged point to Node 12
    pathSystem.addSegment(node12, wait9_south_merged);

    // Warehouse2_access waiting points (505, 1135)
    pathSystem.addSegment(warehouse2_access, wait11_left);
    pathSystem.addSegment(warehouse2_access, wait11_right);

    // Node 12 waiting points (using merged point)
    pathSystem.addSegment(node12, wait12_east);
    pathSystem.addSegment(node12, wait12_west);

    // Node 10 waiting points (using merged point from Node 8)
    pathSystem.addSegment(node10, wait8_10_merged);  // Connect to merged point with Node 8
    pathSystem.addSegment(node10, wait10_left);
    pathSystem.addSegment(node10, wait10_bottom);

    // Node 13 has no waiting points (reverted)

    std::cout << "Factory path system created with " << pathSystem.getNodeCount() << " nodes and " 
              << pathSystem.getSegmentCount() << " segments" << std::endl;
    std::cout << "Added 3 warehouse points and waiting points at all T-junctions" << std::endl;
}