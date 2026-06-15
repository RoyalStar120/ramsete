#pragma once
#include "trajectory.hpp"
#include "lemlib/api.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class RAMSETE {
public:
    RAMSETE(double b, double zeta, double trackWidth, double maxVel, double maxVoltage = 127)
        : b(b), zeta(zeta), trackWidth(trackWidth), maxVel(maxVel), maxVoltage(maxVoltage) {}

    void follow(std::vector<TrajectoryPoint>& trajectory, lemlib::Chassis& chassis) {
        for (auto& point : trajectory) {
            double xd     = point.x;
            double yd     = point.y;
            double thetad = toRadians(point.heading); 
            double vd     = point.velocity;
            double wd     = (point.vRight - point.vLeft) / trackWidth;
            lemlib::Pose pose = chassis.getPose(true, true);
            double x     = pose.x;
            double y     = pose.y;
            double theta = pose.theta;
        
            double dx = xd - x;
            double dy = yd - y;
            double ex =  cos(theta)*dx + sin(theta)*dy;
            double ey = -sin(theta)*dx + cos(theta)*dy;
            double et = thetad - theta;

            // Normalize et to [-π, π]
            while (et >  M_PI) {
                 et -= 2*M_PI;
            }
            while (et < -M_PI) {
                et += 2*M_PI;
            } 
            double k = 2.0 * zeta * sqrt(wd*wd + b * vd*vd);
            double vCmd = vd * cos(et) + k * ex;
            double wCmd = wd + k * et + b * vd * sinc(et) * ey;
            double vLeft  = vCmd - (wCmd * trackWidth) / 2.0;
            double vRight = vCmd + (wCmd * trackWidth) / 2.0;
            double voltLeft  = toVoltage(vLeft);
            double voltRight = toVoltage(vRight);

            voltLeft  = std::max(-maxVoltage, std::min(maxVoltage, voltLeft));
            voltRight = std::max(-maxVoltage, std::min(maxVoltage, voltRight));

            chassis.tank(voltLeft, voltRight);
            pros::delay(10);
        }

        chassis.tank(0, 0);
    }

private:
    double b;
    double zeta;
    double trackWidth;
    double maxVel;
    double maxVoltage;

    double sinc(double x) const {
        if (fabs(x) < 0.001) return 1.0 - (x*x) / 6.0;
        return sin(x) / x;
    }
    double toVoltage(double vel) const {
        return (vel / maxVel) * maxVoltage;
    }
};