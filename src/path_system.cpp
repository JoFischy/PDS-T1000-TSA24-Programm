#include "path_system.h"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <limits>

PathSystem::PathSystem() : nextNodeId(0), nextSegmentId(0) {}

int PathSystem::addNode(float x, float y) {
    PathNode node(nextNodeId, x, y);
    nodes.push_back(node);
    nodeIdToIndex[nextNodeId] = nodes.size() - 1;
    return nextNodeId++;
}

PathNode* PathSystem::getNode(int nodeId) {
    auto it = nodeIdToIndex.find(nodeId);
    return (it != nodeIdToIndex.end()) ? &nodes[it->second] : nullptr;
}

const PathNode* PathSystem::getNode(int nodeId) const {
    auto it = nodeIdToIndex.find(nodeId);
    return (it != nodeIdToIndex.end()) ? &nodes[it->second] : nullptr;
}

int PathSystem::addSegment(int startNodeId, int endNodeId) {
    PathNode* startNode = getNode(startNodeId);
    PathNode* endNode = getNode(endNodeId);
    
    if (!startNode || !endNode) return -1;
    
    // Calculate length
    float length = calculateDistance(startNode->position, endNode->position);
    
    PathSegment segment(nextSegmentId, startNodeId, endNodeId, length);
    segments.push_back(segment);
    segmentIdToIndex[nextSegmentId] = segments.size() - 1;
    
    // Connect nodes to segment
    connectNodeToSegment(startNodeId, nextSegmentId);
    connectNodeToSegment(endNodeId, nextSegmentId);
    
    return nextSegmentId++;
}

PathSegment* PathSystem::getSegment(int segmentId) {
    auto it = segmentIdToIndex.find(segmentId);
    return (it != segmentIdToIndex.end()) ? &segments[it->second] : nullptr;
}

const PathSegment* PathSystem::getSegment(int segmentId) const {
    auto it = segmentIdToIndex.find(segmentId);
    return (it != segmentIdToIndex.end()) ? &segments[it->second] : nullptr;
}

PathSegment* PathSystem::getSegmentBetweenNodes(int nodeId1, int nodeId2) {
    PathNode* node1 = getNode(nodeId1);
    if (!node1) return nullptr;
    
    for (int segmentId : node1->connectedSegments) {
        PathSegment* segment = getSegment(segmentId);
        if (segment && 
            ((segment->startNodeId == nodeId1 && segment->endNodeId == nodeId2) ||
             (segment->startNodeId == nodeId2 && segment->endNodeId == nodeId1))) {
            return segment;
        }
    }
    
    return nullptr;
}

std::vector<int> PathSystem::findPath(int startNodeId, int endNodeId, 
                                     const std::vector<int>& excludedSegments) const {
    std::vector<int> path;
    
    if (startNodeId == endNodeId) return path;
    
    // Create set of excluded segments for fast lookup
    std::unordered_set<int> excludedSet(excludedSegments.begin(), excludedSegments.end());
    
    // Dijkstra's algorithm
    std::unordered_map<int, float> distances;
    std::unordered_map<int, int> previous;
    std::unordered_map<int, int> previousSegment;
    std::priority_queue<std::pair<float, int>, 
                       std::vector<std::pair<float, int>>, 
                       std::greater<std::pair<float, int>>> pq;
    
    // Initialize distances
    for (const auto& node : nodes) {
        distances[node.nodeId] = std::numeric_limits<float>::infinity();
    }
    distances[startNodeId] = 0.0f;
    pq.push({0.0f, startNodeId});
    
    while (!pq.empty()) {
        float currentDist = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();
        
        if (currentNode == endNodeId) break;
        if (currentDist > distances[currentNode]) continue;
        
        const PathNode* node = getNode(currentNode);
        if (!node) continue;
        
        for (int segmentId : node->connectedSegments) {
            // Skip excluded segments
            if (excludedSet.count(segmentId)) continue;
            
            const PathSegment* segment = getSegment(segmentId);
            if (!segment) continue;
            
            // Find the other node
            int otherNode = (segment->startNodeId == currentNode) ? 
                           segment->endNodeId : segment->startNodeId;
            
            float newDist = currentDist + segment->length;
            
            if (newDist < distances[otherNode]) {
                distances[otherNode] = newDist;
                previous[otherNode] = currentNode;
                previousSegment[otherNode] = segmentId;
                pq.push({newDist, otherNode});
            }
        }
    }
    
    // Reconstruct path (as segment IDs)
    if (distances[endNodeId] == std::numeric_limits<float>::infinity()) {
        return path; // No path found
    }
    
    int current = endNodeId;
    while (current != startNodeId) {
        if (previousSegment.find(current) == previousSegment.end()) break;
        path.push_back(previousSegment[current]);
        current = previous[current];
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Point> PathSystem::getPathPoints(const std::vector<int>& segmentIds) const {
    std::vector<Point> points;
    
    if (segmentIds.empty()) return points;
    
    // Add start point of first segment
    const PathSegment* firstSegment = getSegment(segmentIds[0]);
    if (!firstSegment) return points;
    
    const PathNode* startNode = getNode(firstSegment->startNodeId);
    if (startNode) {
        points.push_back(startNode->position);
    }
    
    // Add end points of all segments
    for (int segmentId : segmentIds) {
        const PathSegment* segment = getSegment(segmentId);
        if (!segment) continue;
        
        const PathNode* endNode = getNode(segment->endNodeId);
        if (endNode) {
            points.push_back(endNode->position);
        }
    }
    
    return points;
}

int PathSystem::findNearestNode(const Point& position, float maxDistance) const {
    int nearestNodeId = -1;
    float minDistance = maxDistance;
    
    for (const auto& node : nodes) {
        float distance = calculateDistance(position, node.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNodeId = node.nodeId;
        }
    }
    
    return nearestNodeId;
}

std::vector<int> PathSystem::getConnectedNodes(int nodeId) const {
    std::vector<int> connectedNodes;
    const PathNode* node = getNode(nodeId);
    
    if (!node) return connectedNodes;
    
    for (int segmentId : node->connectedSegments) {
        const PathSegment* segment = getSegment(segmentId);
        if (!segment) continue;
        
        int otherNodeId = (segment->startNodeId == nodeId) ? 
                         segment->endNodeId : segment->startNodeId;
        connectedNodes.push_back(otherNodeId);
    }
    
    return connectedNodes;
}

float PathSystem::getSegmentLength(int segmentId) const {
    const PathSegment* segment = getSegment(segmentId);
    return segment ? segment->length : 0.0f;
}

void PathSystem::connectNodeToSegment(int nodeId, int segmentId) {
    PathNode* node = getNode(nodeId);
    if (node) {
        node->connectedSegments.push_back(segmentId);
    }
}

float PathSystem::calculateDistance(const Point& a, const Point& b) const {
    return a.distanceTo(b);
}