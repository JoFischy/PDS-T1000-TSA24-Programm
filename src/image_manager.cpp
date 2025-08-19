#include "image_manager.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// ImageItem methods
ImageItem::ImageItem() : position({0, 0}), scale(1.0f), rotation(0.0f), isDragging(false), isSelected(false), filename("") {}

void ImageItem::updateBounds() {
    // Bounds based on top-left corner (for collision detection)
    Vector2 topLeft = getTopLeft();
    bounds = {
        topLeft.x,
        topLeft.y,
        texture.width * scale,
        texture.height * scale
    };
}

bool ImageItem::checkPointCollision(Vector2 point) {
    updateBounds();
    // Simplified collision detection - uses bounding box without rotation
    return CheckCollisionPointRec(point, bounds);
}

Vector2 ImageItem::getTopLeft() const {
    return {
        position.x - (texture.width * scale * 0.5f),
        position.y - (texture.height * scale * 0.5f)
    };
}

// ImageManager methods
ImageManager::ImageManager() : selectedImageIndex(-1), currentScenario(0), showUI(true), showImageSelector(false), hoveredImageButton(-1),
                               scenario1Visible(true), scenario2Visible(true), scenario3Visible(true), scenario4Visible(true) {
    // Initialize available images list with ASCII-safe filenames
    availableImages.push_back("assets/Gelbe_Sperrflaeche.png");
    availableImages.push_back("assets/Rote_Sperrflaeche.png");
    availableImages.push_back("assets/VorfahrtGew.png");
    availableImages.push_back("assets/Pfeil geradeaus.png");
    availableImages.push_back("assets/Pfeil_rechts.png");
    availableImages.push_back("assets/Haltelinie.png");
    
    // Load background texture
    backgroundTexture = LoadTexture("assets/factory_layout.png");
    if (backgroundTexture.id == 0) {
        TraceLog(LOG_WARNING, "Could not load background image: assets/factory_layout.png");
        // Create a fallback background
        backgroundTexture = createColorTexture(1920, 1200, LIGHTGRAY);
    }
    
    // Try to load previous configuration
    loadConfiguration("scene_config.txt");
}

ImageManager::~ImageManager() {
    // Save configuration before destroying
    saveConfiguration("scene_config.txt");
    
    // Unload all scenario images
    for (auto& img : scenario1) UnloadTexture(img.texture);
    for (auto& img : scenario2) UnloadTexture(img.texture);
    for (auto& img : scenario3) UnloadTexture(img.texture);
    for (auto& img : scenario4) UnloadTexture(img.texture);
    
    // Unload background texture
    UnloadTexture(backgroundTexture);
}

Texture2D ImageManager::createColorTexture(int width, int height, Color color) {
    Image img = GenImageColor(width, height, color);
    Texture2D texture = LoadTextureFromImage(img);
    UnloadImage(img);
    return texture;
}

void ImageManager::loadImageToCurrentScenario(const std::string& filename) {
    Image img = LoadImage(filename.c_str());
    if (img.data == nullptr) {
        TraceLog(LOG_WARNING, "Could not load image: %s", filename.c_str());
        return;
    }
    
    ImageItem newImage;
    newImage.texture = LoadTextureFromImage(img);
    newImage.filename = filename;
    // Position is now the CENTER of the image
    newImage.position = {(float)GetRandomValue(200, 1400), (float)GetRandomValue(200, 700)};
    newImage.scale = 0.5f; // Start with smaller scale for traffic signs
    newImage.rotation = 0.0f;
    newImage.updateBounds();
    
    // Add to current scenario
    getCurrentScenario().push_back(newImage);
    UnloadImage(img);
    TraceLog(LOG_INFO, "Successfully loaded image to scenario %d: %s", currentScenario, filename.c_str());
}

void ImageManager::clearScenario(int scenarioNumber) {
    std::vector<ImageItem>* targetScenario = nullptr;
    
    switch (scenarioNumber) {
        case 1: targetScenario = &scenario1; break;
        case 2: targetScenario = &scenario2; break;
        case 3: targetScenario = &scenario3; break;
        case 4: targetScenario = &scenario4; break;
        default: return; // Invalid scenario number
    }
    
    // Unload all textures in the scenario
    for (auto& img : *targetScenario) {
        UnloadTexture(img.texture);
    }
    
    // Clear the scenario
    targetScenario->clear();
    
    // Reset selection if we're currently in this scenario
    if (currentScenario == scenarioNumber) {
        selectedImageIndex = -1;
    }
    
    TraceLog(LOG_INFO, "Cleared scenario %d", scenarioNumber);
}

std::vector<ImageItem>& ImageManager::getCurrentScenario() {
    switch (currentScenario) {
        case 1: return scenario1;
        case 2: return scenario2;
        case 3: return scenario3;
        case 4: return scenario4;
        default: 
            // This should never happen in normal operation
            TraceLog(LOG_ERROR, "getCurrentScenario() called with invalid scenario: %d", currentScenario);
            return scenario1; // Fallback to prevent crash
    }
}

void ImageManager::update() {
    Vector2 mousePos = GetMousePosition();
    
    // Handle scenario switching (only in edit modes)
    if (IsKeyPressed(KEY_ONE)) currentScenario = 1;
    if (IsKeyPressed(KEY_TWO)) currentScenario = 2;
    if (IsKeyPressed(KEY_THREE)) currentScenario = 3;
    if (IsKeyPressed(KEY_FOUR)) currentScenario = 4;
    if (IsKeyPressed(KEY_ZERO)) currentScenario = 0; // View all scenarios
    
    // Handle scenario visibility toggles
    if (IsKeyPressed(KEY_F1)) scenario1Visible = !scenario1Visible;
    if (IsKeyPressed(KEY_F2)) scenario2Visible = !scenario2Visible;
    if (IsKeyPressed(KEY_F3)) scenario3Visible = !scenario3Visible;
    if (IsKeyPressed(KEY_F4)) scenario4Visible = !scenario4Visible;
    
    // Handle scenario clearing (CTRL + 1-4)
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        if (IsKeyPressed(KEY_ONE)) clearScenario(1);
        if (IsKeyPressed(KEY_TWO)) clearScenario(2);
        if (IsKeyPressed(KEY_THREE)) clearScenario(3);
        if (IsKeyPressed(KEY_FOUR)) clearScenario(4);
    }
    
    // Get current working scenario
    std::vector<ImageItem>& currentImages = getCurrentScenario();
    
    // In scenario 0 (view mode), don't allow editing
    bool canEdit = (currentScenario != 0);
    
    // Toggle image selector with I key (only in edit modes)
    if (IsKeyPressed(KEY_I) && canEdit) {
        showImageSelector = !showImageSelector;
    }
    
    // Handle image selector
    if (showImageSelector && canEdit) {
        hoveredImageButton = -1;
        
        // Check if mouse is over image selector buttons
        for (int i = 0; i < (int)availableImages.size(); i++) {
            Rectangle buttonRect = {70.0f, 160.0f + i * 60.0f, 360.0f, 50.0f};
            if (CheckCollisionPointRec(mousePos, buttonRect)) {
                hoveredImageButton = i;
                
                if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                    // Load the selected image to current scenario
                    loadImageToCurrentScenario(availableImages[i]);
                    showImageSelector = false; // Close selector after selection
                }
            }
        }
        
        // Don't process other input when selector is open
        return;
    }
    
    // Handle mouse input (only in edit mode)
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && canEdit) {
        selectedImageIndex = -1;
        
        // Check if clicking on any image (from top to bottom)
        for (int i = currentImages.size() - 1; i >= 0; i--) {
            if (currentImages[i].checkPointCollision(mousePos)) {
                selectedImageIndex = i;
                currentImages[i].isDragging = true;
                lastMousePos = mousePos;
                
                // Move selected image to front
                ImageItem temp = currentImages[i];
                currentImages.erase(currentImages.begin() + i);
                currentImages.push_back(temp);
                selectedImageIndex = currentImages.size() - 1;
                break;
            }
        }
        
        // Update selection states
        for (size_t i = 0; i < currentImages.size(); i++) {
            currentImages[i].isSelected = (i == static_cast<size_t>(selectedImageIndex));
        }
    }
    
    if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        for (auto& img : currentImages) {
            img.isDragging = false;
        }
    }
    
    // Handle dragging (only in edit mode)
    if (selectedImageIndex >= 0 && canEdit && currentImages[selectedImageIndex].isDragging) {
        Vector2 delta = {mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y};
        currentImages[selectedImageIndex].position.x += delta.x;
        currentImages[selectedImageIndex].position.y += delta.y;
    }
    
    lastMousePos = mousePos;
    
    // Handle scaling with mouse wheel (only for selected image and in edit mode)
    if (selectedImageIndex >= 0 && canEdit) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                // Rotation when holding SHIFT - 90 degree steps
                if (wheel > 0) {
                    currentImages[selectedImageIndex].rotation += 90.0f;
                } else {
                    currentImages[selectedImageIndex].rotation -= 90.0f;
                }
                // Normalize rotation to 0-360 range
                while (currentImages[selectedImageIndex].rotation >= 360.0f) {
                    currentImages[selectedImageIndex].rotation -= 360.0f;
                }
                while (currentImages[selectedImageIndex].rotation < 0.0f) {
                    currentImages[selectedImageIndex].rotation += 360.0f;
                }
            } else {
                // Scaling when not holding SHIFT - smaller steps and larger range
                currentImages[selectedImageIndex].scale += wheel * 0.05f; // Kleinere Schritte (war 0.1f)
                if (currentImages[selectedImageIndex].scale < 0.05f) {
                    currentImages[selectedImageIndex].scale = 0.05f; // Kleinerer Minimalwert
                }
                if (currentImages[selectedImageIndex].scale > 10.0f) { // Größerer Maximalwert (war 3.0f)
                    currentImages[selectedImageIndex].scale = 10.0f;
                }
            }
        }
    }
    
    // Handle keyboard shortcuts
    if (IsKeyPressed(KEY_TAB)) {
        showUI = !showUI;
    }
    
    if (IsKeyPressed(KEY_DELETE) && selectedImageIndex >= 0 && canEdit) {
        UnloadTexture(currentImages[selectedImageIndex].texture);
        currentImages.erase(currentImages.begin() + selectedImageIndex);
        selectedImageIndex = -1;
    }
}

void ImageManager::render() {
    ClearBackground(RAYWHITE);
    
    // Draw background image instead of grid
    DrawTexturePro(backgroundTexture, 
                   {0, 0, (float)backgroundTexture.width, (float)backgroundTexture.height},
                   {0, 0, 1920, 1200}, 
                   {0, 0}, 0.0f, WHITE);
    
    // Render scenarios based on current mode
    if (currentScenario == 0) {
        // View all scenarios mode
        if (scenario1Visible) renderScenario(scenario1, 1.0f);
        if (scenario2Visible) renderScenario(scenario2, 1.0f);
        if (scenario3Visible) renderScenario(scenario3, 1.0f);
        if (scenario4Visible) renderScenario(scenario4, 1.0f);
    } else {
        // Edit mode - show visible scenarios, with current scenario on top
        if (scenario1Visible && currentScenario != 1) renderScenario(scenario1, 0.7f);
        if (scenario2Visible && currentScenario != 2) renderScenario(scenario2, 0.7f);
        if (scenario3Visible && currentScenario != 3) renderScenario(scenario3, 0.7f);
        if (scenario4Visible && currentScenario != 4) renderScenario(scenario4, 0.7f);
        
        // Render current scenario on top with full opacity
        renderScenario(getCurrentScenario(), 1.0f);
    }
    
    // Draw image selector if active
    if (showImageSelector) {
        drawImageSelector();
    }
    
    // Draw UI
    if (showUI) {
        drawUI();
    }
}

void ImageManager::renderScenario(const std::vector<ImageItem>& scenario, float alpha) {
    for (const auto& img : scenario) {
        // Origin is the center of the scaled image for rotation
        Vector2 origin = {(img.texture.width * img.scale) * 0.5f, (img.texture.height * img.scale) * 0.5f};
        Rectangle sourceRec = {0, 0, (float)img.texture.width, (float)img.texture.height};
        
        // Destination rectangle: center the rectangle at img.position
        Rectangle destRec = {
            img.position.x,  // This will be the center after we set the origin
            img.position.y,  // This will be the center after we set the origin
            img.texture.width * img.scale,
            img.texture.height * img.scale
        };
        
        Color tint = Fade(WHITE, alpha);
        DrawTexturePro(img.texture, sourceRec, destRec, origin, img.rotation, tint);
        
        // Draw selection border only in edit mode and if selected
        if (img.isSelected && currentScenario != 0) {
            // Calculate the four corners of the rotated rectangle
            float halfWidth = (img.texture.width * img.scale) * 0.5f;
            float halfHeight = (img.texture.height * img.scale) * 0.5f;
            
            // Convert rotation to radians
            float radians = img.rotation * DEG2RAD;
            float cosRot = cosf(radians);
            float sinRot = sinf(radians);
            
            // Calculate rotated corners relative to center
            Vector2 corners[4];
            // Top-left
            corners[0].x = img.position.x + (-halfWidth * cosRot - (-halfHeight) * sinRot);
            corners[0].y = img.position.y + (-halfWidth * sinRot + (-halfHeight) * cosRot);
            // Top-right
            corners[1].x = img.position.x + (halfWidth * cosRot - (-halfHeight) * sinRot);
            corners[1].y = img.position.y + (halfWidth * sinRot + (-halfHeight) * cosRot);
            // Bottom-right
            corners[2].x = img.position.x + (halfWidth * cosRot - halfHeight * sinRot);
            corners[2].y = img.position.y + (halfWidth * sinRot + halfHeight * cosRot);
            // Bottom-left
            corners[3].x = img.position.x + (-halfWidth * cosRot - halfHeight * sinRot);
            corners[3].y = img.position.y + (-halfWidth * sinRot + halfHeight * cosRot);
            
            // Draw rotated border lines
            DrawLineEx(corners[0], corners[1], 3, YELLOW); // Top
            DrawLineEx(corners[1], corners[2], 3, YELLOW); // Right
            DrawLineEx(corners[2], corners[3], 3, YELLOW); // Bottom
            DrawLineEx(corners[3], corners[0], 3, YELLOW); // Left
            
            // Draw corner handles at rotated positions
            float handleSize = 8;
            for (int i = 0; i < 4; i++) {
                DrawRectangle(corners[i].x - handleSize/2, corners[i].y - handleSize/2, handleSize, handleSize, YELLOW);
            }
            
            // Draw center point for debugging
            DrawCircle(img.position.x, img.position.y, 3, RED);
        }
    }
}

void ImageManager::drawUI() {
    // Semi-transparent background for UI - größer für mehr Inhalt
    DrawRectangle(10, 10, 450, 590, Fade(BLACK, 0.8f));
    DrawRectangleLines(10, 10, 450, 590, WHITE);
    
    // Title
    DrawText("VERKEHRSZEICHEN MANAGER", 20, 20, 20, YELLOW);
    
    // Current scenario info
    DrawText(TextFormat("AKTUELLES SZENARIO: %d", currentScenario), 20, 50, 16, LIME);
    if (currentScenario == 0) {
        DrawText("(Anzeigemodus - nur betrachten)", 30, 70, 12, LIGHTGRAY);
        int totalImages = scenario1.size() + scenario2.size() + scenario3.size() + scenario4.size();
        DrawText(TextFormat("Gesamt: %d Bilder", totalImages), 30, 85, 11, WHITE);
    } else {
        DrawText("(Bearbeitungsmodus)", 30, 70, 12, LIGHTGRAY);
        int currentImages = getCurrentScenario().size();
        DrawText(TextFormat("Bilder: %d", currentImages), 30, 85, 11, WHITE);
    }
    
    // Scenario controls
    DrawText("SZENARIO WECHSELN:", 20, 110, 14, YELLOW);
    DrawText("• 0 = Alle Szenarien anzeigen", 30, 130, 11, WHITE);
    DrawText("• 1-4 = Szenario bearbeiten", 30, 145, 11, WHITE);
    
    DrawText("SZENARIO SICHTBARKEIT:", 20, 170, 14, YELLOW);
    DrawText("(F1-F4 funktionieren in allen Modi)", 30, 185, 10, LIGHTGRAY);
    DrawText(TextFormat("• F1 = Szenario 1 %s", scenario1Visible ? "AN" : "AUS"), 30, 200, 11, scenario1Visible ? GREEN : RED);
    DrawText(TextFormat("• F2 = Szenario 2 %s", scenario2Visible ? "AN" : "AUS"), 30, 215, 11, scenario2Visible ? GREEN : RED);
    DrawText(TextFormat("• F3 = Szenario 3 %s", scenario3Visible ? "AN" : "AUS"), 30, 230, 11, scenario3Visible ? GREEN : RED);
    DrawText(TextFormat("• F4 = Szenario 4 %s", scenario4Visible ? "AN" : "AUS"), 30, 245, 11, scenario4Visible ? GREEN : RED);
    
    DrawText("SZENARIO LÖSCHEN:", 20, 265, 14, YELLOW);
    DrawText("• STRG + 1-4 = Szenario löschen", 30, 285, 11, WHITE);
    
    // Instructions
    DrawText("STEUERUNG:", 20, 310, 16, LIGHTGRAY);
    if (currentScenario != 0) {
        DrawText("• Klick und ziehen zum Bewegen", 30, 330, 12, WHITE);
        DrawText("• Mausrad zum Skalieren", 30, 345, 12, WHITE);
        DrawText("• SHIFT + Mausrad für 90° Drehung", 30, 360, 12, WHITE);
        DrawText("• I-Taste für Bildauswahl", 30, 375, 12, WHITE);
        DrawText("• DELETE zum Löschen", 30, 390, 12, WHITE);
        DrawText("• Andere Szenarien als Referenz sichtbar", 30, 405, 12, GRAY);
    } else {
        DrawText("• Nur Anzeige - keine Bearbeitung", 30, 330, 12, GRAY);
        DrawText("• Wechsle zu Szenario 1-4 zum Bearbeiten", 30, 345, 12, GRAY);
    }
    DrawText("• TAB um UI ein/auszublenden", 30, 420, 12, WHITE);
    DrawText("• ESC zum Beenden", 30, 435, 12, WHITE);
    
    // Available images
    DrawText("VERFÜGBARE BILDER:", 20, 460, 14, YELLOW);
    DrawText("• Gelbe Sperrfläche", 30, 480, 11, WHITE);
    DrawText("• Rote Sperrfläche", 30, 495, 11, WHITE);
    DrawText("• Vorfahrt gewähren", 30, 510, 11, WHITE);
    DrawText("• Pfeil geradeaus", 30, 525, 11, WHITE);
    DrawText("• Pfeil rechts", 30, 540, 11, WHITE);
    DrawText("• Haltelinie", 30, 555, 11, WHITE);
}

void ImageManager::drawImageSelector() {
    // Semi-transparent overlay
    DrawRectangle(0, 0, 1920, 1200, Fade(BLACK, 0.7f));
    
    // Main selector background - größer für mehr Bilder
    DrawRectangle(50, 100, 400, 450, Fade(DARKGRAY, 0.95f));
    DrawRectangleLines(50, 100, 400, 450, WHITE);
    
    // Title
    DrawText("VERKEHRSZEICHEN AUSWÄHLEN", 70, 120, 18, YELLOW);
    DrawText(TextFormat("Verfügbare Bilder: %d", (int)availableImages.size()), 70, 140, 12, LIGHTGRAY);
    
    // Image selection buttons
    for (int i = 0; i < (int)availableImages.size(); i++) {
        Rectangle buttonRect = {70.0f, 160.0f + i * 60.0f, 360.0f, 50.0f};
        
        // Button background
        Color buttonColor = (hoveredImageButton == i) ? Fade(BLUE, 0.8f) : Fade(GRAY, 0.6f);
        DrawRectangleRec(buttonRect, buttonColor);
        DrawRectangleLinesEx(buttonRect, 2, WHITE);
        
        // Button text
        std::string displayName;
        if (i == 0) displayName = "Gelbe Sperrfläche";
        else if (i == 1) displayName = "Rote Sperrfläche"; 
        else if (i == 2) displayName = "Vorfahrt gewähren";
        else if (i == 3) displayName = "Pfeil geradeaus";
        else if (i == 4) displayName = "Pfeil rechts";
        else if (i == 5) displayName = "Haltelinie";
        else displayName = "Unbekannt"; // Fallback für zusätzliche Bilder
        
        DrawText(displayName.c_str(), buttonRect.x + 20, buttonRect.y + 15, 16, WHITE);
    }
    
    // Instructions
    DrawText("Klicke auf ein Bild zum Hinzufügen", 70, 500, 14, LIGHTGRAY);
    DrawText("Drücke I zum Schließen", 70, 520, 14, LIGHTGRAY);
}

void ImageManager::saveConfiguration(const std::string& filename) {
    FILE* file = fopen(filename.c_str(), "w");
    if (!file) {
        TraceLog(LOG_WARNING, "Could not save configuration to: %s", filename.c_str());
        return;
    }
    
    // Save visibility states
    fprintf(file, "VISIBILITY %d %d %d %d\n", 
            scenario1Visible ? 1 : 0,
            scenario2Visible ? 1 : 0,
            scenario3Visible ? 1 : 0,
            scenario4Visible ? 1 : 0);
    
    // Save each scenario
    auto saveScenario = [&](const std::vector<ImageItem>& scenario, int scenarioNum) {
        fprintf(file, "SCENARIO %d %d\n", scenarioNum, (int)scenario.size());
        for (const auto& img : scenario) {
            fprintf(file, "%s %.2f %.2f %.2f %.2f\n", 
                    img.filename.c_str(),
                    img.position.x, img.position.y,
                    img.scale, img.rotation);
        }
    };
    
    saveScenario(scenario1, 1);
    saveScenario(scenario2, 2);
    saveScenario(scenario3, 3);
    saveScenario(scenario4, 4);
    
    fclose(file);
    TraceLog(LOG_INFO, "Configuration saved to: %s", filename.c_str());
}

void ImageManager::loadConfiguration(const std::string& filename) {
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        TraceLog(LOG_INFO, "No previous configuration found: %s", filename.c_str());
        return;
    }
    
    // Clear existing scenarios
    for (auto& img : scenario1) UnloadTexture(img.texture);
    for (auto& img : scenario2) UnloadTexture(img.texture);
    for (auto& img : scenario3) UnloadTexture(img.texture);
    for (auto& img : scenario4) UnloadTexture(img.texture);
    scenario1.clear();
    scenario2.clear();
    scenario3.clear();
    scenario4.clear();
    
    char line[1024];
    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "VISIBILITY", 10) == 0) {
            int vis1, vis2, vis3, vis4;
            sscanf(line, "VISIBILITY %d %d %d %d", &vis1, &vis2, &vis3, &vis4);
            scenario1Visible = (vis1 != 0);
            scenario2Visible = (vis2 != 0);
            scenario3Visible = (vis3 != 0);
            scenario4Visible = (vis4 != 0);
        } else if (strncmp(line, "SCENARIO", 8) == 0) {
            int scenarioNum, imageCount;
            sscanf(line, "SCENARIO %d %d", &scenarioNum, &imageCount);
            
            std::vector<ImageItem>* targetScenario = nullptr;
            switch (scenarioNum) {
                case 1: targetScenario = &scenario1; break;
                case 2: targetScenario = &scenario2; break;
                case 3: targetScenario = &scenario3; break;
                case 4: targetScenario = &scenario4; break;
                default: continue;
            }
            
            for (int i = 0; i < imageCount; i++) {
                if (!fgets(line, sizeof(line), file)) break;
                
                char filename_buffer[512];
                float x, y, scale, rotation;
                
                if (sscanf(line, "%511s %f %f %f %f", filename_buffer, &x, &y, &scale, &rotation) == 5) {
                    // Try to load the image
                    Image img = LoadImage(filename_buffer);
                    if (img.data != nullptr) {
                        ImageItem newImage;
                        newImage.texture = LoadTextureFromImage(img);
                        newImage.filename = filename_buffer;
                        newImage.position = {x, y};
                        newImage.scale = scale;
                        newImage.rotation = rotation;
                        newImage.updateBounds();
                        
                        targetScenario->push_back(newImage);
                        UnloadImage(img);
                        TraceLog(LOG_INFO, "Loaded image to scenario %d: %s", scenarioNum, filename_buffer);
                    } else {
                        TraceLog(LOG_WARNING, "Could not load image from config: %s", filename_buffer);
                    }
                }
            }
        }
    }
    
    fclose(file);
    TraceLog(LOG_INFO, "Configuration loaded from: %s", filename.c_str());
}
