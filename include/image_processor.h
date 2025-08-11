#pragma once
#include <vector>

struct PixelColor {
    unsigned char r, g, b, a;
    
    PixelColor() : r(0), g(0), b(0), a(255) {}
    PixelColor(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha = 255)
        : r(red), g(green), b(blue), a(alpha) {}
    
    bool isWhite(int threshold = 200) const {
        return r >= threshold && g >= threshold && b >= threshold;
    }
    
    bool isBlack(int threshold = 50) const {
        return r <= threshold && g <= threshold && b <= threshold;
    }
    
    int brightness() const {
        return (r + g + b) / 3;
    }
};

class ImageProcessor {
public:
    ImageProcessor();
    ~ImageProcessor();
    
    // File operations
    bool loadImage(const char* filename);
    void cleanup();
    
    // Pixel access
    PixelColor getPixel(int x, int y) const;
    void setPixel(int x, int y, const PixelColor& color);
    
    // Image analysis
    std::vector<std::vector<bool>> createBinaryMask(int threshold = 200) const;
    
    // Image processing operations
    void applyGaussianBlur(int radius = 1);
    void threshold(int thresholdValue = 128);
    void erode(int radius = 1);
    void dilate(int radius = 1);
    void opening(int radius = 1);
    void closing(int radius = 1);
    
    // Getters
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getChannels() const { return channels; }
    const unsigned char* getData() const { return imageData; }
    
private:
    bool isValidCoordinate(int x, int y) const;
    
    unsigned char* imageData;
    int width;
    int height;
    int channels;
};