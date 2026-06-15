#pragma once
#include "primitives.hpp"
#include <vector>
#include <cmath>

// cubic hermite spline between 2 waypoints

class SplineSegment {
public:
    SplineSegment(WayPoint start, WayPoint end)
        : start(start), end(end) {}

    std::vector<SplinePoint> sample(int samples) const {
        std::vector<SplinePoint> points;
        for (int i=0; i<samples; i++) {
            double u = (double)i / samples;
            double px = x(u);
            double py = y(u);
            double xd = xPrime(u);
            double yd = yPrime(u);
            double xdd = xDoublePrime(u);
            double ydd = yDoublePrime(u);
            double heading = toDegrees(atan2(yd, xd));
            double k = curvature(xd, yd, xdd, ydd);
            points.push_back({px, py, heading, k});
        }
        return points;
    }
    
private:
    WayPoint start;
    WayPoint end;

    double tangentScale() const {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        return sqrt(dx*dx + dy*dy);
    }

    //position
    double x(double u) const {
        double s = tangentScale();
        double t0 = cos(toRadians(start.heading))*s;
        double t1 = cos(toRadians(end.heading))*s;
        return (2*u*u*u - 3*u*u + 1)*start.x
        +(u*u*u - 2*u*u + u)*t0
        +(-2*u*u*u + 3*u*u)*end.x
        +(u*u*u - u*u)*t1;
    }

    double y(double u) const {
        double s = tangentScale();
        double t0 = sin(toRadians(start.heading))*s;
        double t1 = sin(toRadians(end.heading))*s;
        return (2*u*u*u - 3*u*u + 1)*start.y
        +(u*u*u - 2*u*u + u)*t0
        +(-2*u*u*u + 3*u*u)*end.y
        +(u*u*u - u*u)*t1;
    }

    //first derivatives
    double xPrime(double u) const {
        double s = tangentScale();
        double t0 = cos(toRadians(start.heading))*s;
        double t1 = cos(toRadians(end.heading))*s;
        return (6*u*u - 6*u)*start.x
        +(3*u*u - 4*u + 1)*t0
        +(-6*u*u + 6*u)*end.x
        +(3*u*u - 2*u)*t1;
    }

    double yPrime(double u) const {
        double s = tangentScale();
        double t0 = sin(toRadians(start.heading))*s;
        double t1 = sin(toRadians(end.heading))*s;
        return (6*u*u - 6*u)*start.y
        +(3*u*u - 4*u + 1)*t0
        +(-6*u*u + 6*u)*end.y
        +(3*u*u - 2*u)*t1;
    }

    //second derivatives
    double xDoublePrime(double u) const {
        double s = tangentScale();
        double t0 = cos(toRadians(start.heading))*s;
        double t1 = cos(toRadians(end.heading))*s;
        return (12*u - 6)*start.x
        +(6*u - 4)*t0
        +(-12*u + 6)*end.x
        +(6*u - 2)*t1;
    }

    double yDoublePrime(double u) const {
        double s = tangentScale();
        double t0 = sin(toRadians(start.heading))*s;
        double t1 = sin(toRadians(end.heading))*s;
        return (12*u - 6)*start.y
        +(6*u - 4)*t0
        +(-12*u + 6)*end.y
        +(6*u - 2)*t1;
    }

    double curvature(double xd, double yd, double xdd, double ydd) const {
        double denom = pow(xd*xd + yd*yd, 1.5);
        if (fabs(denom) < 1e-9) {
            return 0.0;
        }
        return (xd*ydd - yd*xdd)/ denom;
    }

};
