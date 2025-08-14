#pragma once
#include "path_system.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <memory>

class SegmentManager {
public:
    SegmentManager(PathSystem* pathSys);

    // Segmentreservierung
    bool reserveSegment(int segmentId, int vehicleId);
    void releaseSegment(int segmentId, int vehicleId);
    bool canVehicleEnterSegment(int segmentId, int vehicleId) const;

    // Warteschlangenmanagement
    void addToQueue(int segmentId, int vehicleId);
    void removeFromQueue(int segmentId, int vehicleId);
    void updateQueues();

    // Fahrzeugverfolgung
    void removeVehicle(int vehicleId);
    int getVehicleSegment(int vehicleId) const;

    // Hilfsmethoden
    bool isSegmentOccupied(int segmentId) const;
    int getSegmentOccupant(int segmentId) const;

private:
    PathSystem* pathSystem;
    std::unordered_map<int, int> segmentOccupancy; // segmentId -> vehicleId
    std::unordered_map<int, std::queue<int>> segmentQueues; // segmentId -> queue of vehicleIds
    std::unordered_map<int, int> vehicleToSegment; // vehicleId -> segmentId
};