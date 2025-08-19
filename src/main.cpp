#include "raylib.h"
#include "image_manager.h"

int main() {
    // Get monitor size and initialize fullscreen window
    int monitor = GetCurrentMonitor();
    int screenWidth = GetMonitorWidth(monitor);
    int screenHeight = GetMonitorHeight(monitor);
    
    InitWindow(screenWidth, screenHeight, "PDS-T1000 Szenario Manager");
    ToggleFullscreen(); // Start in fullscreen
    SetTargetFPS(60);
    
    ImageManager manager;
    
    while (!WindowShouldClose()) {
        manager.update();
        
        BeginDrawing();
        manager.render();
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}