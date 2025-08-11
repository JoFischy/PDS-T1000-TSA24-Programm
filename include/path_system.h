#pragma once
#include "point.h"
#include <vector>
#include <unordered_map>

struct PathNode {
    int nodeId;
    Point position;
    std::vector<int> connectedSegments;
    
    PathNode() : nodeId(-1) {}
    PathNode(int id, float x, float y) : nodeId(id), position(x, y) {}
};

struct PathSegment {
    int segmentId;
    int startNodeId;
    int endNodeId;
    float length;
    bool isOccupied;
    int occupiedByVehicleId;
    std::vector<int> queuedVehicles;
    
    PathSegment() : segmentId(-1), startNodeId(-1), endNodeId(-1), 
                   length(0), isOccupied(false), occupiedByVehicleId(-1) {}
    PathSegment(int id, int start, int end, float len) 
        : segmentId(id), startNodeId(start), endNodeId(end), length(len),
          isOccupied(false), occupiedByVehicleId(-1) {}
};

class PathSystem {
public:
    PathSystem();
    
    // Node management
    int addNode(float x, float y);
    PathNode* getNode(int nodeId);
    const PathNode* getNode(int nodeId) const;
    
    // Segment management
    int addSegment(int startNodeId, int endNodeId);
    PathSegment* getSegment(int segmentId);
    const PathSegment* getSegment(int segmentId) const;
    PathSegment* getSegmentBetweenNodes(int nodeId1, int nodeId2);
    
    // Path finding
    std::vector<int> findPath(int startNodeId, int endNodeId, 
                             const std::vector<int>& excludedSegments = {}) const;
    std::vector<Point> getPathPoints(const std::vector<int>& segmentIds) const;
    
    // Utilities
    int findNearestNode(const Point& position, float maxDistance = 50.0f) const;
    std::vector<int> getConnectedNodes(int nodeId) const;
    float getSegmentLength(int segmentId) const;
    
    // Getters
    const std::vector<PathNode>& getNodes() const { return nodes; }
    const std::vector<PathSegment>& getSegments() const { return segments; }
    size_t getNodeCount() const { return nodes.size(); }
    size_t getSegmentCount() const { return segments.size(); }
    
private:
    std::vector<PathNode> nodes;
    std::vector<PathSegment> segments;
    std::unordered_map<int, size_t> nodeIdToIndex;
    std::unordered_map<int, size_t> segmentIdToIndex;
    int nextNodeId;
    int nextSegmentId;
    
    void connectNodeToSegment(int nodeId, int segmentId);
    float calculateDistance(const Point& a, const Point& b) const;
};