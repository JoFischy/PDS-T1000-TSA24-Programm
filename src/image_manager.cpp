#include "image_manager.h"
#include <cmath>

// ImageItem methods
ImageItem::ImageItem() : position({0, 0}), scale(1.0f), rotation(0.0f), isDragging(false), isSelected(false), filename("") {}

void ImageItem::updateBounds() {
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
    return CheckCollisionPointRec(point, bounds);
}

Vector2 ImageItem::getTopLeft() const {
    return {
        position.x - (texture.width * scale * 0.5f),
        position.y - (texture.height * scale * 0.5f)
    };
}

// ImageManager methods
ImageManager::ImageManager() : selectedImageIndex(-1), currentScenario(0), showUI(true), 
                               showImageSelector(false), hoveredImageButton(-1),
                               scenario1Visible(false), scenario2Visible(false), 
                               scenario3Visible(false), scenario4Visible(false) {
    // Initialize available images list
    availableImages.push_back("assets/Gelbe_Sperrflaeche.png");
    availableImages.push_back("assets/Rote_Sperrflaeche.png");
    availableImages.push_back("assets/VorfahrtGew.png");
    availableImages.push_back("assets/Haltelinie.png");
    
    // Load background texture
    Image bgImg = LoadImage("assets/factory_layout.png");
    if (bgImg.data != nullptr) {
        // Scale to screen size
        int screenWidth = GetScreenWidth();
        int screenHeight = GetScreenHeight();
        ImageResize(&bgImg, screenWidth, screenHeight);
        backgroundTexture = LoadTextureFromImage(bgImg);
        UnloadImage(bgImg);
        TraceLog(LOG_INFO, "Successfully loaded factory_layout.png");
    } else {
        // Create fallback texture
        backgroundTexture = createColorTexture(GetScreenWidth(), GetScreenHeight(), DARKGRAY);
        TraceLog(LOG_WARNING, "Could not load factory_layout.png, using fallback");
    }
}

ImageManager::~ImageManager() {
    // Unload textures from all scenarios
    for (auto& img : scenario1) UnloadTexture(img.texture);
    for (auto& img : scenario2) UnloadTexture(img.texture);
    for (auto& img : scenario3) UnloadTexture(img.texture);
    for (auto& img : scenario4) UnloadTexture(img.texture);
    
    UnloadTexture(backgroundTexture);
}

Texture2D ImageManager::createColorTexture(int width, int height, Color color) {
    Image img = GenImageColor(width, height, color);
    Texture2D texture = LoadTextureFromImage(img);
    UnloadImage(img);
    return texture;
}

std::vector<ImageItem>& ImageManager::getCurrentScenario() {
    switch (currentScenario) {
        case 1: return scenario1;
        case 2: return scenario2;
        case 3: return scenario3;
        case 4: return scenario4;
        case 0: // Anzeigemodus - return dummy (wird nicht verwendet)
        default: return scenario1;
    }
}

bool ImageManager::loadImageFromFile(const std::string& filename) {
    Image img = LoadImage(filename.c_str());
    if (img.data == nullptr) {
        TraceLog(LOG_WARNING, "Could not load image: %s", filename.c_str());
        return false;
    }
    
    ImageItem newImage;
    newImage.texture = LoadTextureFromImage(img);
    newImage.filename = filename;
    newImage.position = {(float)GetRandomValue(300, GetScreenWidth()-300), (float)GetRandomValue(200, GetScreenHeight()-200)};
    newImage.scale = 0.5f;
    newImage.rotation = 0.0f;
    newImage.updateBounds();
    
    getCurrentScenario().push_back(newImage);
    UnloadImage(img);
    TraceLog(LOG_INFO, "Successfully loaded image: %s to scenario %d", filename.c_str(), currentScenario);
    return true;
}

void ImageManager::update() {
    Vector2 mousePos = GetMousePosition();
    
    // Scenario switching with number keys
    if (IsKeyPressed(KEY_ZERO)) {
        currentScenario = 0; // Anzeigemodus
        selectedImageIndex = -1;
    }
    if (IsKeyPressed(KEY_ONE)) {
        currentScenario = 1;
        selectedImageIndex = -1;
    }
    if (IsKeyPressed(KEY_TWO)) {
        currentScenario = 2;
        selectedImageIndex = -1;
    }
    if (IsKeyPressed(KEY_THREE)) {
        currentScenario = 3;
        selectedImageIndex = -1;
    }
    if (IsKeyPressed(KEY_FOUR)) {
        currentScenario = 4;
        selectedImageIndex = -1;
    }
    
    // Toggle scenario visibility with F keys
    if (IsKeyPressed(KEY_F1)) scenario1Visible = !scenario1Visible;
    if (IsKeyPressed(KEY_F2)) scenario2Visible = !scenario2Visible;
    if (IsKeyPressed(KEY_F3)) scenario3Visible = !scenario3Visible;
    if (IsKeyPressed(KEY_F4)) scenario4Visible = !scenario4Visible;
    
    // Im Anzeigemodus (0) keine Bearbeitung erlauben
    if (currentScenario == 0) {
        // Nur UI-Toggle erlauben
        if (IsKeyPressed(KEY_TAB)) {
            showUI = !showUI;
        }
        return; // Keine weitere Interaktion im Anzeigemodus
    }
    
    // Toggle image selector with I key (nur in Bearbeitungsmodi)
    if (IsKeyPressed(KEY_I)) {
        showImageSelector = !showImageSelector;
    }
    
    // Handle image selector
    if (showImageSelector) {
        hoveredImageButton = -1;
        
        for (int i = 0; i < (int)availableImages.size(); i++) {
            Rectangle buttonRect = {70.0f, 160.0f + i * 60.0f, 450.0f, 50.0f};
            if (CheckCollisionPointRec(mousePos, buttonRect)) {
                hoveredImageButton = i;
                
                if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                    if (loadImageFromFile(availableImages[i])) {
                        showImageSelector = false;
                    }
                }
            }
        }
        return; // Don't process other input when selector is open
    }
    
    // Handle mouse input for current scenario
    auto& currentImages = getCurrentScenario();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
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
    
    // Handle dragging
    if (selectedImageIndex >= 0 && selectedImageIndex < currentImages.size() && currentImages[selectedImageIndex].isDragging) {
        Vector2 delta = {mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y};
        currentImages[selectedImageIndex].position.x += delta.x;
        currentImages[selectedImageIndex].position.y += delta.y;
    }
    
    lastMousePos = mousePos;
    
    // Handle scaling with mouse wheel (only for selected image)
    if (selectedImageIndex >= 0 && selectedImageIndex < currentImages.size()) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            if (IsKeyDown(KEY_LEFT_SHIFT)) {
                // Rotation when holding SHIFT - 90 degree steps
                currentImages[selectedImageIndex].rotation += wheel * 90.0f;
                if (currentImages[selectedImageIndex].rotation >= 360.0f) {
                    currentImages[selectedImageIndex].rotation -= 360.0f;
                }
                if (currentImages[selectedImageIndex].rotation < 0.0f) {
                    currentImages[selectedImageIndex].rotation += 360.0f;
                }
            } else {
                // Scaling when not holding SHIFT
                currentImages[selectedImageIndex].scale += wheel * 0.1f;
                if (currentImages[selectedImageIndex].scale < 0.1f) {
                    currentImages[selectedImageIndex].scale = 0.1f;
                }
                if (currentImages[selectedImageIndex].scale > 3.0f) {
                    currentImages[selectedImageIndex].scale = 3.0f;
                }
            }
        }
    }
    
    // Handle keyboard shortcuts
    if (IsKeyPressed(KEY_TAB)) {
        showUI = !showUI;
    }
    
    if (IsKeyPressed(KEY_DELETE) && selectedImageIndex >= 0 && selectedImageIndex < currentImages.size()) {
        UnloadTexture(currentImages[selectedImageIndex].texture);
        currentImages.erase(currentImages.begin() + selectedImageIndex);
        selectedImageIndex = -1;
    }
}

void ImageManager::renderScenario(const std::vector<ImageItem>& scenario, float alpha) {
    for (const auto& img : scenario) {
        Vector2 origin = {(img.texture.width * img.scale) * 0.5f, (img.texture.height * img.scale) * 0.5f};
        Rectangle sourceRec = {0, 0, (float)img.texture.width, (float)img.texture.height};
        
        Rectangle destRec = {
            img.position.x,
            img.position.y,
            img.texture.width * img.scale,
            img.texture.height * img.scale
        };
        
        Color tint = Fade(WHITE, alpha);
        DrawTexturePro(img.texture, sourceRec, destRec, origin, img.rotation, tint);
        
        // Draw selection border only for current scenario
        if (alpha >= 1.0f && img.isSelected) {
            float halfWidth = (img.texture.width * img.scale) * 0.5f;
            float halfHeight = (img.texture.height * img.scale) * 0.5f;
            
            float radians = img.rotation * DEG2RAD;
            float cosRot = cosf(radians);
            float sinRot = sinf(radians);
            
            Vector2 corners[4];
            corners[0].x = img.position.x + (-halfWidth * cosRot - (-halfHeight) * sinRot);
            corners[0].y = img.position.y + (-halfWidth * sinRot + (-halfHeight) * cosRot);
            corners[1].x = img.position.x + (halfWidth * cosRot - (-halfHeight) * sinRot);
            corners[1].y = img.position.y + (halfWidth * sinRot + (-halfHeight) * cosRot);
            corners[2].x = img.position.x + (halfWidth * cosRot - halfHeight * sinRot);
            corners[2].y = img.position.y + (halfWidth * sinRot + halfHeight * cosRot);
            corners[3].x = img.position.x + (-halfWidth * cosRot - halfHeight * sinRot);
            corners[3].y = img.position.y + (-halfWidth * sinRot + halfHeight * cosRot);
            
            DrawLineEx(corners[0], corners[1], 3, YELLOW);
            DrawLineEx(corners[1], corners[2], 3, YELLOW);
            DrawLineEx(corners[2], corners[3], 3, YELLOW);
            DrawLineEx(corners[3], corners[0], 3, YELLOW);
            
            float handleSize = 8;
            for (int i = 0; i < 4; i++) {
                DrawRectangle(corners[i].x - handleSize/2, corners[i].y - handleSize/2, handleSize, handleSize, YELLOW);
            }
            
            DrawCircle(img.position.x, img.position.y, 3, RED);
        }
    }
}

void ImageManager::render() {
    ClearBackground(BLACK);
    
    // Draw background image (factory layout) - FULL SCREEN
    Rectangle sourceRec = {0, 0, (float)backgroundTexture.width, (float)backgroundTexture.height};
    Rectangle destRec = {0, 0, (float)GetScreenWidth(), (float)GetScreenHeight()};
    DrawTexturePro(backgroundTexture, sourceRec, destRec, {0, 0}, 0.0f, WHITE);
    
    // Render visible scenarios with transparency
    if (scenario1Visible) renderScenario(scenario1, 0.7f);
    if (scenario2Visible) renderScenario(scenario2, 0.7f);
    if (scenario3Visible) renderScenario(scenario3, 0.7f);
    if (scenario4Visible) renderScenario(scenario4, 0.7f);
    
    // Render current scenario with full opacity (nur wenn nicht im Anzeigemodus)
    if (currentScenario != 0) {
        renderScenario(getCurrentScenario(), 1.0f);
    }
    
    // Draw image selector if active (nur wenn nicht im Anzeigemodus)
    if (showImageSelector && currentScenario != 0) {
        drawImageSelector();
    }
    
    // Draw UI
    if (showUI) {
        drawUI();
    }
    
    // INFOLEISTE ENTFERNT - keine Status-Bar mehr
}

void ImageManager::drawUI() {
    int uiWidth = 520;
    int uiHeight = 480;
    DrawRectangle(10, 10, uiWidth, uiHeight, Fade(BLACK, 0.8f));
    DrawRectangleLines(10, 10, uiWidth, uiHeight, WHITE);
    
    DrawText("PDS-T1000 SZENARIO MANAGER", 20, 20, 18, YELLOW);
    
    // Current mode display
    if (currentScenario == 0) {
        DrawText("MODUS: ANZEIGE (nur betrachten)", 20, 50, 14, ORANGE);
    } else {
        DrawText(TextFormat("MODUS: BEARBEITUNG - Szenario %d", currentScenario), 20, 50, 14, LIME);
    }
    
    DrawText("TASTEN-STEUERUNG:", 20, 80, 12, YELLOW);
    DrawText("• 0 - Anzeigemodus (nur betrachten)", 30, 100, 11, ORANGE);
    DrawText("• 1,2,3,4 - Szenario bearbeiten", 30, 115, 11, WHITE);
    DrawText("• F1,F2,F3,F4 - Szenario ein/ausblenden", 30, 130, 11, WHITE);
    
    // Show scenario status
    DrawText("STATUS:", 20, 160, 12, YELLOW);
    Color s1Color = (currentScenario == 1) ? LIME : (scenario1Visible ? LIGHTGRAY : GRAY);
    Color s2Color = (currentScenario == 2) ? LIME : (scenario2Visible ? LIGHTGRAY : GRAY);
    Color s3Color = (currentScenario == 3) ? LIME : (scenario3Visible ? LIGHTGRAY : GRAY);
    Color s4Color = (currentScenario == 4) ? LIME : (scenario4Visible ? LIGHTGRAY : GRAY);
    
    DrawText(TextFormat("Szenario 1: %s (%d Bilder)", scenario1Visible ? "Sichtbar" : "Verborgen", (int)scenario1.size()), 30, 180, 10, s1Color);
    DrawText(TextFormat("Szenario 2: %s (%d Bilder)", scenario2Visible ? "Sichtbar" : "Verborgen", (int)scenario2.size()), 30, 195, 10, s2Color);
    DrawText(TextFormat("Szenario 3: %s (%d Bilder)", scenario3Visible ? "Sichtbar" : "Verborgen", (int)scenario3.size()), 30, 210, 10, s3Color);
    DrawText(TextFormat("Szenario 4: %s (%d Bilder)", scenario4Visible ? "Sichtbar" : "Verborgen", (int)scenario4.size()), 30, 225, 10, s4Color);
    
    // Instructions - only show editing controls if not in display mode
    if (currentScenario != 0) {
        DrawText("BEARBEITUNGS-STEUERUNG:", 20, 255, 12, YELLOW);
        DrawText("• Klick und ziehen zum Bewegen", 30, 275, 11, WHITE);
        DrawText("• Mausrad zum Skalieren", 30, 290, 11, WHITE);
        DrawText("• SHIFT + Mausrad zum Drehen", 30, 305, 11, WHITE);
        DrawText("• I-Taste für Bildauswahl", 30, 320, 11, WHITE);
        DrawText("• DELETE zum Löschen", 30, 335, 11, WHITE);
        
        // Current selection info
        auto& currentImages = getCurrentScenario();
        if (selectedImageIndex >= 0 && selectedImageIndex < currentImages.size()) {
            const auto& img = currentImages[selectedImageIndex];
            DrawText("AUSGEWÄHLT:", 20, 365, 11, YELLOW);
            DrawText(TextFormat("Datei: %s", img.filename.substr(img.filename.find_last_of("/\\") + 1).c_str()), 30, 385, 9, WHITE);
            DrawText(TextFormat("Position: (%.0f, %.0f) | Größe: %.1fx | Winkel: %.0f°", 
                     img.position.x, img.position.y, img.scale, img.rotation), 30, 400, 9, WHITE);
        } else {
            DrawText("Kein Bild ausgewählt - Drücke I für Auswahl", 30, 385, 10, GRAY);
        }
    } else {
        DrawText("ANZEIGEMODUS:", 20, 255, 12, YELLOW);
        DrawText("• Nur F1-F4 zum Ein-/Ausblenden", 30, 275, 11, WHITE);
        DrawText("• Keine Bearbeitung möglich", 30, 290, 11, WHITE);
        DrawText("• Wechsle zu 1-4 zum Bearbeiten", 30, 305, 11, WHITE);
    }
    
    DrawText("ALLGEMEIN:", 20, 435, 12, YELLOW);
    DrawText("• TAB um UI ein/auszublenden", 30, 450, 11, WHITE);
    DrawText("• ESC zum Beenden", 30, 465, 11, WHITE);
}

void ImageManager::drawImageSelector() {
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.7f));
    
    DrawRectangle(50, 100, 500, 410, Fade(DARKGRAY, 0.95f));
    DrawRectangleLines(50, 100, 500, 410, WHITE);
    
    DrawText("VERKEHRSZEICHEN AUSWÄHLEN", 70, 120, 18, YELLOW);
    DrawText(TextFormat("Für Szenario %d", currentScenario), 70, 145, 14, LIGHTGRAY);
    
    for (int i = 0; i < (int)availableImages.size(); i++) {
        Rectangle buttonRect = {70.0f, 180.0f + i * 60.0f, 450.0f, 50.0f};
        
        Color buttonColor = (hoveredImageButton == i) ? Fade(BLUE, 0.8f) : Fade(GRAY, 0.6f);
        DrawRectangleRec(buttonRect, buttonColor);
        DrawRectangleLinesEx(buttonRect, 2, WHITE);
        
        std::string displayName;
        if (i == 0) displayName = "Gelbe Sperrfläche";
        else if (i == 1) displayName = "Rote Sperrfläche"; 
        else if (i == 2) displayName = "Vorfahrt gewähren";
        else if (i == 3) displayName = "Haltelinie";
        
        DrawText(displayName.c_str(), buttonRect.x + 20, buttonRect.y + 15, 16, WHITE);
    }
    
    DrawText("Klicke auf ein Bild zum Hinzufügen", 70, 440, 14, LIGHTGRAY);
    DrawText("Drücke I zum Schließen", 70, 460, 14, LIGHTGRAY);
}
