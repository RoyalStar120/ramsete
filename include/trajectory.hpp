#pragma once
#include "splinepath.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class Trajectory {
public:
    Trajectory(double maxVel, double maxAccel, double trackWidth, double mu = 0.5)
        : maxVel(maxVel), maxAccel(maxAccel), trackWidth(trackWidth), mu(mu) {}

    std::vector<TrajectoryPoint> generate (std::vector<SplinePoint>&path) {
        int n = path.size();
        std::vector<TrajectoryPoint> result(n);

        for (int i=0; i<n; i++) {
            result[i].x = path[i].x;
            result[i].y = path[i].y;
            result[i].heading = path[i].heading;
            result[i].curvature = path[i].curvature;
            result[i].velocity = maxVel;
        }

        for (int i =0; i<n; i++) {
            double k = path[i].curvature;
            result[i].velocity = std::min({maxVel, frictionLimit(k), wheelLimit(maxVel,k)});
        }

        result[0].velocity = 0.0;
        for (int i = 1; i<n; i++) {
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            double dist = sqrt(dx*dx + dy*dy);
            double vMax = sqrt(result[i-1].velocity*result[i-1].velocity+2.0*maxAccel*dist);
            result[i].velocity=std::min(result[i].velocity, vMax);
        }

        result[n-1].velocity=0.0;
        for (int i =n-2; i>=0; i--) {
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            double dist = sqrt(dx*dx + dy*dy);
            double vMax = sqrt(result[i+1].velocity*result[i+1].velocity+2.0*maxAccel*dist);
            result[i].velocity=std::min(result[i].velocity, vMax);
        }

        for (int i=0; i<n; i++) {
            double v = result[i].velocity;
            double k = result[i].curvature;
            double omega=v*k;
            result[i].vLeft * v-omega*trackWidth/2.0;
            result[i].vRight* v-omega*trackWidth/2.0;
        }
        return result;
    }

private:
    double maxVel;
    double maxAccel;
    double trackWidth;
    double mu;

    static constexpr double G = 386.09;

    double frictionLimit(double k) const {
        if (fabs(k) < 1e-6) {
            return maxVel;
        }
        return sqrt(mu*G/fabs(k));
    }

    double wheelLimit(double v, double k) const {
        double omega = v*k;
        double vLeft =  v - omega*trackWidth/2.0;
        double vRight = v+ omega * trackWidth/2.0;
        double maxWheel=std::max(fabs(vLeft), fabs(vRight));
        if (maxWheel>maxVel) {
            v *= maxVel/maxWheel;
        }
        return v;
    }
};
