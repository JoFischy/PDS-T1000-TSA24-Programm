#include "image_processor.h"
#include "raylib.h"
#include <algorithm>
#include <cstring>

ImageProcessor::ImageProcessor() : imageData(nullptr), width(0), height(0), channels(0) {}

ImageProcessor::~ImageProcessor() {
    cleanup();
}

bool ImageProcessor::loadImage(const char* filename) {
    Image img = LoadImage(filename);
    if (img.data == nullptr) return false;
    
    // Convert to RGBA format for consistency
    ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    
    width = img.width;
    height = img.height;
    channels = 4; // RGBA
    
    // Allocate and copy image data
    size_t dataSize = width * height * channels;
    imageData = new unsigned char[dataSize];
    memcpy(imageData, img.data, dataSize);
    
    UnloadImage(img);
    return true;
}

void ImageProcessor::cleanup() {
    if (imageData) {
        delete[] imageData;
        imageData = nullptr;
    }
    width = height = channels = 0;
}

PixelColor ImageProcessor::getPixel(int x, int y) const {
    if (!isValidCoordinate(x, y)) return PixelColor();
    
    int index = (y * width + x) * channels;
    return PixelColor(imageData[index], imageData[index + 1], 
                      imageData[index + 2], imageData[index + 3]);
}

void ImageProcessor::setPixel(int x, int y, const PixelColor& color) {
    if (!isValidCoordinate(x, y)) return;
    
    int index = (y * width + x) * channels;
    imageData[index] = color.r;
    imageData[index + 1] = color.g;
    imageData[index + 2] = color.b;
    imageData[index + 3] = color.a;
}

std::vector<std::vector<bool>> ImageProcessor::createBinaryMask(int threshold) const {
    std::vector<std::vector<bool>> mask(height, std::vector<bool>(width, false));
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            PixelColor pixel = getPixel(x, y);
            mask[y][x] = pixel.isWhite(threshold);
        }
    }
    
    return mask;
}

void ImageProcessor::applyGaussianBlur(int radius) {
    if (!imageData || radius <= 0) return;
    
    unsigned char* tempData = new unsigned char[width * height * channels];
    memcpy(tempData, imageData, width * height * channels);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int totalR = 0, totalG = 0, totalB = 0, totalA = 0;
            int count = 0;
            
            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int index = (ny * width + nx) * channels;
                        totalR += tempData[index];
                        totalG += tempData[index + 1];
                        totalB += tempData[index + 2];
                        totalA += tempData[index + 3];
                        count++;
                    }
                }
            }
            
            if (count > 0) {
                int index = (y * width + x) * channels;
                imageData[index] = totalR / count;
                imageData[index + 1] = totalG / count;
                imageData[index + 2] = totalB / count;
                imageData[index + 3] = totalA / count;
            }
        }
    }
    
    delete[] tempData;
}

void ImageProcessor::threshold(int thresholdValue) {
    if (!imageData) return;
    
    for (int i = 0; i < width * height * channels; i += channels) {
        int brightness = (imageData[i] + imageData[i + 1] + imageData[i + 2]) / 3;
        unsigned char value = (brightness >= thresholdValue) ? 255 : 0;
        
        imageData[i] = value;     // R
        imageData[i + 1] = value; // G
        imageData[i + 2] = value; // B
        // Keep alpha unchanged
    }
}

void ImageProcessor::erode(int radius) {
    if (!imageData || radius <= 0) return;
    
    unsigned char* tempData = new unsigned char[width * height * channels];
    memcpy(tempData, imageData, width * height * channels);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            bool isWhite = true;
            
            // Check neighborhood
            for (int dy = -radius; dy <= radius && isWhite; dy++) {
                for (int dx = -radius; dx <= radius && isWhite; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int index = (ny * width + nx) * channels;
                        if (tempData[index] < 128) { // Not white
                            isWhite = false;
                        }
                    } else {
                        isWhite = false; // Border pixels become black
                    }
                }
            }
            
            int index = (y * width + x) * channels;
            unsigned char value = isWhite ? 255 : 0;
            imageData[index] = value;
            imageData[index + 1] = value;
            imageData[index + 2] = value;
        }
    }
    
    delete[] tempData;
}

void ImageProcessor::dilate(int radius) {
    if (!imageData || radius <= 0) return;
    
    unsigned char* tempData = new unsigned char[width * height * channels];
    memcpy(tempData, imageData, width * height * channels);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            bool hasWhite = false;
            
            // Check neighborhood
            for (int dy = -radius; dy <= radius && !hasWhite; dy++) {
                for (int dx = -radius; dx <= radius && !hasWhite; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int index = (ny * width + nx) * channels;
                        if (tempData[index] >= 128) { // White pixel found
                            hasWhite = true;
                        }
                    }
                }
            }
            
            int index = (y * width + x) * channels;
            unsigned char value = hasWhite ? 255 : 0;
            imageData[index] = value;
            imageData[index + 1] = value;
            imageData[index + 2] = value;
        }
    }
    
    delete[] tempData;
}

void ImageProcessor::opening(int radius) {
    erode(radius);
    dilate(radius);
}

void ImageProcessor::closing(int radius) {
    dilate(radius);
    erode(radius);
}

bool ImageProcessor::isValidCoordinate(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}