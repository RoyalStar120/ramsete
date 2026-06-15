#pragma once
#include "splinesegment.hpp"
#include <vector>

class SplinePath {
public:
    SplinePath(int samplesPerSegment = 50)
        : samplesPerSegment(samplesPerSegment) {}

    std::vector<SplinePoint> generate(std::vector<WayPoint> waypoints) {
        std::vector<SplinePoint> fullPath;

        for (int i=0; i<(int)waypoints.size() - 1; i++) {
            SplineSegment segment(waypoints[i], waypoints[i+1]);
            auto segPoints = segment.sample(samplesPerSegment);
            fullPath.insert(fullPath.end(), segPoints.begin(), segPoints.end());
        }

        WayPoint&last = waypoints.back();
        fullPath.push_back({last.x, last.y, last.heading, 0.0});
        return fullPath;
    }

private:
    int samplesPerSegment;
};