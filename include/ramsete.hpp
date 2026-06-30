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
        int n = trajectory.size();
        std::vector<uint32_t> timestamps(n);
        timestamps[0] = 0;
        for (int i = 1; i < n; i++) {
            double dx     = trajectory[i].x - trajectory[i-1].x;
            double dy     = trajectory[i].y - trajectory[i-1].y;
            double dist   = sqrt(dx*dx + dy*dy);
            double avgVel = (trajectory[i].velocity + trajectory[i-1].velocity) / 2.0;
            // dt in ms = (distance / velocity) * 1000
            double dt = (avgVel > 1e-6) ? (dist / avgVel) * 1000.0 : 0.0;
            timestamps[i] = timestamps[i-1] + (uint32_t)dt;
        }
        uint32_t startTime = pros::millis();

        while (true) {
            uint32_t elapsed = pros::millis() - startTime;
            int idx = n-1;
            for (int i = 0; i < n - 1; i++) {
                if (timestamps[i] >= elapsed) {
                    idx = i;
                    break;
                }
            }

            TrajectoryPoint& point = trajectory[idx];
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

            if (elapsed >= timestamps[n - 1]) {
                break;
            }
            
            uint32_t loopTime = pros::millis() - (startTime + elapsed);
            if (loopTime < 10) {
                pros::delay(10 - loopTime);
            }
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
        if (fabs(x) < 0.001) {
            return 1.0 - (x*x) / 6.0;
        }
        return sin(x) / x;
    }

    double toVoltage(double vel) const {
        return (vel / maxVel) * maxVoltage;
    }
};