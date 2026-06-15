#pragma once
#include <cmath>

struct Pose {
    double x;
    double y;;
    double theta;
};

struct WayPoint {
    double x;
    double y;
    double heading;
};

struct SplinePoint {
    double x;
    double y;
    double heading;
    double curvature;
};

struct TrajectoryPoint {
    double x;
    double y;
    double heading;
    double curvature;
    double velocity;
    double vLeft;
    double vRight;
};

inline double toRadians(double deg) {return deg*M_PI/180.0;}
inline double toDegrees(double rad) {return rad*180/M_PI;}