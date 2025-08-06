
#ifndef MOVEMENT_SYSTEM_H
#define MOVEMENT_SYSTEM_H

#include "point.h"
#include <vector>

enum class Direction {
    NORTH = 0,
    EAST = 90,
    SOUTH = 180,
    WEST = 270
};

struct PathNode {
    Point position;
    Direction direction;
    
    PathNode(float x, float y, Direction dir) 
        : position(x, y), direction(dir) {}
};

// Abstract interface for movement commands
class MovementInterface {
public:
    virtual ~MovementInterface() = default;
    virtual void moveForward(int vehicleId, float distance) = 0;
    virtual void turnLeft(int vehicleId) = 0;
    virtual void turnRight(int vehicleId) = 0;
    virtual void setPosition(int vehicleId, const Point& position, Direction direction) = 0;
};

// Path management system
class PathSystem {
private:
    std::vector<PathNode> pathNodes;
    
public:
    void addNode(float x, float y, Direction direction);
    void clearPath();
    const std::vector<PathNode>& getPath() const { return pathNodes; }
    PathNode getClosestNode(const Point& position) const;
    PathNode getNextNode(size_t currentIndex) const;
    size_t getPathSize() const { return pathNodes.size(); }
};

#endif
