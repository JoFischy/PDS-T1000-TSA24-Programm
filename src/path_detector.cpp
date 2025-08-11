#include "path_detector.h"
#include "image_processor.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <cstring>

PathDetector::PathDetector() : imageData{nullptr, 0, 0, 0} {}

PathDetector::~PathDetector() {
    cleanup();
}

bool PathDetector::loadImage(const char* imagePath) {
    ImageProcessor processor;
    if (!processor.loadImage(imagePath)) {
        return false;
    }
    
    imageData.width = processor.getWidth();
    imageData.height = processor.getHeight();
    imageData.channels = processor.getChannels();
    
    // Copy image data
    size_t dataSize = imageData.width * imageData.height * imageData.channels;
    imageData.pixels = new unsigned char[dataSize];
    memcpy(imageData.pixels, processor.getData(), dataSize);
    
    return true;
}

bool PathDetector::processImage() {
    if (!imageData.pixels) return false;
    
    // Find all white path pixels
    pathPoints = findPathPixels();
    if (pathPoints.empty()) return false;
    
    // Detect intersections
    std::vector<Point> rawIntersections = detectIntersections(pathPoints);
    
    // Optimize and merge nearby intersections
    intersections = optimizeIntersections(rawIntersections);
    
    return !intersections.empty();
}

bool PathDetector::isWhitePixel(int x, int y, int threshold) const {
    if (x < 0 || x >= imageData.width || y < 0 || y >= imageData.height) {
        return false;
    }
    
    int index = (y * imageData.width + x) * imageData.channels;
    
    if (imageData.channels >= 3) {
        unsigned char r = imageData.pixels[index];
        unsigned char g = imageData.pixels[index + 1];
        unsigned char b = imageData.pixels[index + 2];
        return r >= threshold && g >= threshold && b >= threshold;
    } else if (imageData.channels == 1) {
        return imageData.pixels[index] >= threshold;
    }
    
    return false;
}

std::vector<Point> PathDetector::findPathPixels(int threshold) const {
    std::vector<Point> pathPixels;
    
    // Use smaller step for better precision with factory layouts
    int step = 3;
    for (int y = 0; y < imageData.height; y += step) {
        for (int x = 0; x < imageData.width; x += step) {
            if (isWhitePixel(x, y, threshold)) {
                pathPixels.emplace_back(x, y);
            }
        }
    }
    
    return pathPixels;
}

std::vector<Point> PathDetector::detectIntersections(const std::vector<Point>& pathPixels) const {
    std::vector<Point> intersections;
    
    // Create a set for fast lookup
    std::unordered_set<long long> pathPixelSet;
    for (const Point& p : pathPixels) {
        pathPixelSet.insert(static_cast<long long>(p.x) * 1000000 + static_cast<long long>(p.y));
    }
    
    // Check each path pixel to see if it's an intersection
    for (const Point& pixel : pathPixels) {
        if (isIntersection(static_cast<int>(pixel.x), static_cast<int>(pixel.y), pathPixels)) {
            intersections.push_back(pixel);
        }
    }
    
    return intersections;
}

bool PathDetector::isIntersection(int x, int y, const std::vector<Point>& pathPixels) const {
    // Count the number of path neighbors in different directions
    int directionCount = 0;
    
    // Check 8 directions around the pixel
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    std::vector<bool> hasNeighbor(8, false);
    
    for (int i = 0; i < 8; i++) {
        int nx = x + dx[i] * 3; // Check a bit further out
        int ny = y + dy[i] * 3;
        
        if (isWhitePixel(nx, ny)) {
            hasNeighbor[i] = true;
        }
    }
    
    // Count distinct direction groups
    // Group adjacent directions: N, NE, E, SE, S, SW, W, NW
    std::vector<bool> directionGroups(4, false);
    
    // North group (N, NE, NW)
    if (hasNeighbor[1] || hasNeighbor[0] || hasNeighbor[2]) directionGroups[0] = true;
    // East group (E, NE, SE)  
    if (hasNeighbor[7] || hasNeighbor[0] || hasNeighbor[3]) directionGroups[1] = true;
    // South group (S, SE, SW)
    if (hasNeighbor[4] || hasNeighbor[3] || hasNeighbor[5]) directionGroups[2] = true;
    // West group (W, SW, NW)
    if (hasNeighbor[6] || hasNeighbor[5] || hasNeighbor[2]) directionGroups[3] = true;
    
    int activeGroups = 0;
    for (bool active : directionGroups) {
        if (active) activeGroups++;
    }
    
    // An intersection should have at least 3 direction groups
    return activeGroups >= 3;
}

int PathDetector::countNeighbors(int x, int y, const std::vector<Point>& pathPixels, int radius) const {
    int count = 0;
    
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            if (isWhitePixel(x + dx, y + dy)) {
                count++;
            }
        }
    }
    
    return count;
}

std::vector<Point> PathDetector::optimizeIntersections(const std::vector<Point>& rawIntersections) const {
    std::vector<Point> optimized = rawIntersections;
    
    // Merge nearby intersections
    mergeNearbyIntersections(optimized);
    
    return optimized;
}

void PathDetector::mergeNearbyIntersections(std::vector<Point>& intersections, float mergeRadius) const {
    bool merged = true;
    
    while (merged) {
        merged = false;
        
        for (size_t i = 0; i < intersections.size() && !merged; i++) {
            for (size_t j = i + 1; j < intersections.size(); j++) {
                if (intersections[i].distanceTo(intersections[j]) < mergeRadius) {
                    // Merge j into i (average position)
                    intersections[i].x = (intersections[i].x + intersections[j].x) / 2.0f;
                    intersections[i].y = (intersections[i].y + intersections[j].y) / 2.0f;
                    
                    // Remove j
                    intersections.erase(intersections.begin() + j);
                    merged = true;
                    break;
                }
            }
        }
    }
}

PathSystem PathDetector::generatePathSystem() const {
    PathSystem pathSystem;
    
    // Add all intersections as nodes
    std::vector<int> nodeIds;
    for (const Point& intersection : intersections) {
        int nodeId = pathSystem.addNode(intersection.x, intersection.y);
        nodeIds.push_back(nodeId);
    }
    
    // Connect nearby nodes with segments
    float maxConnectionDistance = 150.0f; // Increased for factory layouts
    
    for (size_t i = 0; i < intersections.size(); i++) {
        for (size_t j = i + 1; j < intersections.size(); j++) {
            float distance = intersections[i].distanceTo(intersections[j]);
            
            if (distance <= maxConnectionDistance) {
                // Check if there's a clear path between the intersections
                if (hasPathBetweenPoints(intersections[i], intersections[j])) {
                    pathSystem.addSegment(nodeIds[i], nodeIds[j]);
                }
            }
        }
    }
    
    return pathSystem;
}

bool PathDetector::hasPathBetweenPoints(const Point& start, const Point& end) const {
    // Simple line-based path detection
    float distance = start.distanceTo(end);
    if (distance < 1.0f) return true;
    
    int steps = static_cast<int>(distance);
    Point direction = Point((end.x - start.x) / distance, (end.y - start.y) / distance);
    
    int whitePixelCount = 0;
    int totalSamples = 0;
    
    for (int i = 1; i < steps; i++) {
        Point samplePoint = Point(start.x + direction.x * i, start.y + direction.y * i);
        totalSamples++;
        
        if (isWhitePixel(static_cast<int>(samplePoint.x), static_cast<int>(samplePoint.y))) {
            whitePixelCount++;
        }
    }
    
    // Require at least 70% of sampled points to be white (more tolerant)
    return totalSamples > 0 && (static_cast<float>(whitePixelCount) / totalSamples) >= 0.7f;
}

void PathDetector::cleanup() {
    if (imageData.pixels) {
        delete[] imageData.pixels;
        imageData.pixels = nullptr;
    }
    imageData.width = imageData.height = imageData.channels = 0;
    pathPoints.clear();
    intersections.clear();
}