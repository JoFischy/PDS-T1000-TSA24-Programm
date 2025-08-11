#pragma once
#include "point.h"
#include "path_system.h"
#include <vector>

struct ImageData {
    unsigned char* pixels;
    int width;
    int height;
    int channels;
};

class PathDetector {
public:
    PathDetector();
    ~PathDetector();
    
    // Main interface
    bool loadImage(const char* imagePath);
    bool processImage();
    PathSystem generatePathSystem() const;
    
    // Path detection methods
    std::vector<Point> findPathPixels(int threshold = 200) const;
    std::vector<Point> detectIntersections(const std::vector<Point>& pathPixels) const;
    std::vector<Point> optimizeIntersections(const std::vector<Point>& rawIntersections) const;
    
    // Utility methods
    bool isWhitePixel(int x, int y, int threshold = 200) const;
    bool isIntersection(int x, int y, const std::vector<Point>& pathPixels) const;
    int countNeighbors(int x, int y, const std::vector<Point>& pathPixels, int radius = 2) const;
    bool hasPathBetweenPoints(const Point& start, const Point& end) const;
    
    // Getters
    const std::vector<Point>& getPathPoints() const { return pathPoints; }
    const std::vector<Point>& getIntersections() const { return intersections; }
    const ImageData& getImageData() const { return imageData; }
    
private:
    void cleanup();
    void mergeNearbyIntersections(std::vector<Point>& intersections, float mergeRadius = 20.0f) const;
    
    ImageData imageData;
    std::vector<Point> pathPoints;
    std::vector<Point> intersections;
};