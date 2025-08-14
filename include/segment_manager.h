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

    // Section-based Reservierung
    bool reserveSection(const std::vector<int>& sectionSegments, int vehicleId);
    void releaseSection(const std::vector<int>& sectionSegments, int vehicleId);
    bool canVehicleEnterSection(const std::vector<int>& sectionSegments, int vehicleId) const;

    // Legacy Segmentreservierung (für Kompatibilität)
    bool reserveSegment(int segmentId, int vehicleId);
    void releaseSegment(int segmentId, int vehicleId);
    bool canVehicleEnterSegment(int segmentId, int vehicleId) const;

    // Section-Erkennung
    std::vector<int> getSectionSegments(int startNodeId, int endNodeId) const;

    // Warteschlangenmanagement
    void addToQueue(int segmentId, int vehicleId);
    void removeFromQueue(int segmentId, int vehicleId);
    void updateQueues();

    // Fahrzeugverfolgung
    void removeVehicle(int vehicleId);
    int getVehicleSegment(int vehicleId) const;
    std::vector<int> getVehicleSection(int vehicleId) const;

    // Hilfsmethoden
    bool isSegmentOccupied(int segmentId) const;
    int getSegmentOccupant(int segmentId) const;

    // Build section from segment to next T-section
    std::vector<int> buildSectionFromSegment(int startSegmentId) const;
    int findNearestTSection(int startNodeId, int excludeSegmentId) const;

private:
    PathSystem* pathSystem;
    std::unordered_map<int, int> segmentOccupancy; // segmentId -> vehicleId
    std::unordered_map<int, std::queue<int>> segmentQueues; // segmentId -> queue of vehicleIds
    std::unordered_map<int, int> vehicleToSegment; // vehicleId -> segmentId
    std::unordered_map<int, std::vector<int>> vehicleToSection; // vehicleId -> section segments
};