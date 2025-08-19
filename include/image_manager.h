#pragma once
#include "raylib.h"
#include <vector>
#include <string>

struct ImageItem {
    Texture2D texture;
    Vector2 position;
    float scale;
    float rotation;
    bool isDragging;
    bool isSelected;
    std::string filename;
    Rectangle bounds;
    
    ImageItem();
    void updateBounds();
    bool checkPointCollision(Vector2 point);
    Vector2 getTopLeft() const;
};

class ImageManager {
private:
    std::vector<ImageItem> scenario1;
    std::vector<ImageItem> scenario2;
    std::vector<ImageItem> scenario3;
    std::vector<ImageItem> scenario4;
    
    std::vector<std::string> availableImages;
    int selectedImageIndex;
    int currentScenario; // 0 = Anzeigemodus, 1-4 = Bearbeitungsmodi
    Vector2 lastMousePos;
    bool showUI;
    bool showImageSelector;
    int hoveredImageButton;
    
    bool scenario1Visible;
    bool scenario2Visible;
    bool scenario3Visible;
    bool scenario4Visible;
    
    Texture2D backgroundTexture;
    
    Texture2D createColorTexture(int width, int height, Color color);
    bool loadImageFromFile(const std::string& filename);
    void drawUI();
    void drawImageSelector();
    std::vector<ImageItem>& getCurrentScenario();
    void renderScenario(const std::vector<ImageItem>& scenario, float alpha = 1.0f);
    
public:
    ImageManager();
    ~ImageManager();
    
    void update();
    void render();
};
