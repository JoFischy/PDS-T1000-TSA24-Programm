#include "raylib.h"
#include <iostream>

int main() {
    InitWindow(800, 600, "Image Test");
    SetTargetFPS(60);
    
    // Test image loading
    Image img1 = LoadImage("assets/Gelbe_Sperrfläche.png");
    Image img2 = LoadImage("assets/Rote_Sperrfläche.png");
    Image img3 = LoadImage("assets/VorfahrtGew.png");
    
    std::cout << "Gelbe_Sperrfläche: " << (img1.data ? "OK" : "FAILED") << std::endl;
    std::cout << "Rote_Sperrfläche: " << (img2.data ? "OK" : "FAILED") << std::endl;
    std::cout << "VorfahrtGew: " << (img3.data ? "OK" : "FAILED") << std::endl;
    
    if (img1.data) {
        Texture2D tex1 = LoadTextureFromImage(img1);
        UnloadImage(img1);
        
        while (!WindowShouldClose()) {
            BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawTexture(tex1, 100, 100, WHITE);
            DrawText("Test: Gelbe Sperrfläche", 10, 10, 20, BLACK);
            EndDrawing();
        }
        
        UnloadTexture(tex1);
    } else {
        while (!WindowShouldClose()) {
            BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawText("Could not load image!", 10, 10, 20, RED);
            EndDrawing();
        }
    }
    
    if (img2.data) UnloadImage(img2);
    if (img3.data) UnloadImage(img3);
    
    CloseWindow();
    return 0;
}
