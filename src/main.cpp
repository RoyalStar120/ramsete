#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "primitives.hpp"
#include "splinepath.hpp"
#include "trajectory.hpp"
#include "ramsete.hpp"
#include "dsr.hpp"
#include <algorithm> // for std::clamp

const double TRACK_WIDTH = 10.75;
const double MAX_VEL     = 59;
const double MAX_ACCEL   = 60;
const double MU          = 0.5;
const double B           = 2.0;
const double ZETA        = 0.7;


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-4, -5, 6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({1, 2, -3}, pros::MotorGearset::blue);

// sensors
pros::Imu imu(17);

// mechanisms
pros::Motor lift(15);
pros::Motor intake(-20);
pros::Rotation liftrot(-12);
pros::Motor claw(13);
pros::Motor clawrotator(14);
pros::Rotation clawrot(-16);

// odometry
pros::Rotation verticalEnc(7);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.875);

// drivetrain
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.75, lemlib::Omniwheel::NEW_325, 360, 2);

// lateral controller
lemlib::ControllerSettings linearController(6, 0, 28, 3, 1, 100, 3, 500, 0);

// angular controller
lemlib::ControllerSettings angularController(4, 0, 33, 3, 1, 100, 3, 500, 0);

// sensors
lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


pros::Distance left_dist(18);
pros::Distance front_dist(10);
DsrSensor left_dsr(&left_dist, -5.375, 0.46875, 270, 15);
DsrSensor front_dsr(&front_dist, -1.6875, 2.7375, 0, 15);
DsrTracking DsrMain(&chassis, 20, false, 100, 100, 10.0, 6.0, 20);




// trajectory
SplinePath splinePath(50);
Trajectory trajectory(MAX_VEL, MAX_ACCEL, TRACK_WIDTH, MU);
RAMSETE ramsete(B, ZETA, TRACK_WIDTH, MAX_VEL);

void followPath(std::vector<WayPoint> waypoints) {
    auto splinePoints = splinePath.generate(waypoints);
    auto trajectoryPoints = trajectory.generate(splinePoints);
    ramsete.follow(trajectoryPoints, chassis);
}

double startpos;
double clawstart;

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    
    startpos = liftrot.get_position();
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawrotator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawstart = clawrot.get_position();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "liftrot: %d", liftrot.get_position());
            pros::lcd::print(4, "clawrot: %d", clawrot.get_position());
            pros::lcd::print(5, "claw rpm: %f", claw.get_actual_velocity());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

ASSET(example_txt);

std::vector<WayPoint> myPath = {
    {0, 0, 90},
    {24, 24, 45},
    {48, 0, 0}
};

void autonomous() {
    chassis.setPose(-118, 118, 0);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    DsrMain.updateBotPose(&front_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
}

bool movingarm = false;
bool movingclaw = false;
const double increment = 3000;
const double maxincrements = 5;
double targetpos;
double currentpos;
double clawcurrent;
double clawtarget;
uint32_t clawMoveStart = 0;
uint32_t clawRestartTime = 0;
bool clawDisabled = false;
int clawDirection = 0;

const double CLAW_MIN_RPM = 140;
const uint32_t CLAW_SPINUP_TIME = 400;
const uint32_t CLAW_RESTART_DELAY = 300;

void setClaw(int power) {
    if (power != 0 && power != clawDirection) {
        clawMoveStart = pros::millis();
        clawDirection = power;
    }

    if (!clawDisabled) {
        claw.move(power);
    }
}

lemlib::PID liftPID(0.1, 0, 0.02, 3000, true);
lemlib::PID clawPID(0.02, 0, 0.06, 3000, true);
const int CLAW_TEST_POWER = 70;
const uint32_t CLAW_JAM_WAIT = 200;

//helpers

void driveLiftTo(double target, uint32_t timeoutMs = 2000) {
    liftPID.reset();
    uint32_t start = pros::millis();

    while (pros::millis() - start < timeoutMs) {
        double current = liftrot.get_position();
        double error = target - current;

        double output = liftPID.update(error);
        output = std::clamp(output, -127.0, 127.0);

        lift.move(output);

        if (std::fabs(error) < 50) break;
        pros::delay(10);
    }
    lift.brake();
    liftPID.reset();
}

void driveClawRotatorTo(double target, uint32_t timeoutMs = 2000) {
    clawPID.reset();
    uint32_t start = pros::millis();

    while (pros::millis() - start < timeoutMs) {
        double current = clawrot.get_position();
        double error = target - current;

        double output = clawPID.update(error);
        output = std::clamp(output, -127.0, 127.0);

        clawrotator.move(output);

        if (std::fabs(error) < 50) break;
        pros::delay(10);
    }
    clawrotator.brake();
    clawPID.reset();
}

//macro 1: lift to `amount` increments, then the open/rotate-up thingajamiggygy

void liftToAmount(int amount) {
    double maxpos = startpos + maxincrements * increment;
    double target = startpos + amount * increment;
    target = std::clamp(target, startpos, maxpos);

    driveLiftTo(target);
    pros::delay(500);
    setClaw(127); // spin claw outward
    pros::delay(500);
    driveClawRotatorTo(clawstart + 51000); // rotate claw up to max
}

//macro 2: claw rotator all the way down + arm to startpos

void resetLiftAndClaw() {
    liftPID.reset();
    clawPID.reset();
    bool liftDone = false;
    bool clawDone = false;
    uint32_t start = pros::millis();

    while (pros::millis() - start < 2000 && !(liftDone && clawDone)) {
        if (!liftDone) {
            double liftError = startpos - liftrot.get_position();
            double liftOutput = liftPID.update(liftError);
            liftOutput = std::clamp(liftOutput, -127.0, 127.0);
            lift.move(liftOutput);

            if (std::fabs(liftError) < 50) {
                lift.brake();
                liftDone = true;
            }
        }

        if (!clawDone) {
            double clawError = clawstart - clawrot.get_position();
            double clawOutput = clawPID.update(clawError);
            clawOutput = std::clamp(clawOutput, -127.0, 127.0);
            clawrotator.move(clawOutput);

            if (std::fabs(clawError) < 50) {
                clawrotator.brake();
                clawDone = true;
            }
        }
        pros::delay(10);
    }
    lift.brake();
    clawrotator.brake();
    liftPID.reset();
    clawPID.reset();
}

void opcontrol() {
    clawMoveStart = pros::millis();
    bool intakeToggle = false;
    currentpos = startpos;
    targetpos = startpos;
    clawcurrent = clawstart;
    clawtarget = clawstart;
    const double maxpos = startpos + maxincrements * increment;
    uint32_t startTime = 0;
    bool timerStarted = false;
    bool wasHoldingA = false;

    while (true) {
        currentpos = liftrot.get_position();
        clawcurrent = clawrot.get_position();

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && targetpos > startpos) {
            targetpos -= increment;
            movingarm = true;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && targetpos < maxpos) {
            if (targetpos == startpos) {
                targetpos += 500;
            } else {
                targetpos += increment;
            }
            movingarm = true;

            if (clawtarget < clawstart + 51000) {
                clawtarget = clawstart + 51000;
                movingclaw = true;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            targetpos = startpos;
            movingarm = true;
            clawtarget = clawstart;
            movingclaw = true;
        }

        if (!clawDisabled) {
            setClaw(-127);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            wasHoldingA = true;

            if (!timerStarted) {
                timerStarted = true;
                startTime = pros::millis();
                clawtarget = clawstart + 33500;
                movingclaw = true;
            }

            if (pros::millis() - startTime >= 500) {
                setClaw(127);
            }
        }
        else if (wasHoldingA) {
            wasHoldingA = false;
            timerStarted = false;
            clawtarget = clawstart + 51000;
            movingclaw = true;
            setClaw(-127);
        }

        if (!clawDisabled) {
            // wait for spinup before checking
            if (pros::millis() - clawMoveStart > CLAW_SPINUP_TIME) {
                if (std::fabs(claw.get_actual_velocity()) < CLAW_MIN_RPM) {
                    // detected jam
                    claw.move(0);
                    clawDisabled = true;
                    // start the waiting period
                    clawRestartTime = pros::millis();
                }
            }
        }
        else {
            // wait 150ms after detecting jam before testing again
            if (pros::millis() - clawRestartTime > CLAW_JAM_WAIT) {
                // try spinning briefly
                // gently test if jam cleared
                claw.move((clawDirection > 0) ? CLAW_TEST_POWER : -CLAW_TEST_POWER);
                pros::delay(100);

                if (std::fabs(claw.get_actual_velocity()) >= CLAW_MIN_RPM) {
                    // jam cleared
                    clawDisabled = false;
                    clawMoveStart = pros::millis();
                }
                else {
                    // still jammed
                    claw.move(0);
                    // restart wait timer
                    clawRestartTime = pros::millis();
                }
            }
        }

        if (movingclaw) {
            double error = clawtarget - clawcurrent;
            double output = clawPID.update(error);

            if (output > 127) output = 127;
            if (output < -127) output = -127;

            clawrotator.move(output);

            if (std::fabs(error) < 50) {
                clawrotator.brake();
                movingclaw = false;
                clawPID.reset();
            }
        }

        if (movingarm) {
            double error = targetpos - currentpos;
            double output = liftPID.update(error);

            if (output > 127) output = 127;
            if (output < -127) output = -127;

            lift.move(output);

            if (std::fabs(error) < 50) {
                lift.brake();
                movingarm = false;
                liftPID.reset();
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeToggle = !intakeToggle;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);
        }
        else if (intakeToggle) {
            intake.move(-127);
        }
        else {
            intake.move(0);
        }
        pros::delay(10);
    }
}
