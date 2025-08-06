
#include "movement_system.h"
#include <cmath>
#include <limits>

void PathSystem::addNode(float x, float y, Direction direction) {
    pathNodes.emplace_back(x, y, direction);
}

void PathSystem::clearPath() {
    pathNodes.clear();
}

PathNode PathSystem::getClosestNode(const Point& position) const {
    if (pathNodes.empty()) {
        return PathNode(position.x, position.y, Direction::NORTH);
    }
    
    size_t closestIndex = 0;
    float minDistance = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < pathNodes.size(); i++) {
        float distance = position.distanceTo(pathNodes[i].position);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    
    return pathNodes[closestIndex];
}

PathNode PathSystem::getNextNode(size_t currentIndex) const {
    if (pathNodes.empty() || currentIndex >= pathNodes.size() - 1) {
        return pathNodes.empty() ? PathNode(0, 0, Direction::NORTH) : pathNodes.back();
    }
    return pathNodes[currentIndex + 1];
}
