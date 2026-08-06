#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj_class.h"
#include "pros/device.hpp"
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

bool intakeToggle = false;

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
pros::Distance right_dist(11);
DsrSensor left_dsr(&left_dist, -5.375, 0.46875, 270, 15);
DsrSensor front_dsr(&front_dist, -1.6875, 2.7375, 0, 15);
DsrSensor right_dsr(&right_dist, 6.625, 0.46875, 90, 15);
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



void disabled() {}

void competition_initialize() {}

ASSET(example_txt);

std::vector<WayPoint> myPath = {
    {0, 0, 90},
    {24, 24, 45},
    {48, 0, 0}
};



const double increment = 3350;
const double maxincrements = 4;
double targetpos;
double currentpos;
double clawcurrent;
double clawtarget;
uint32_t clawMoveStart = 0;
uint32_t clawRestartTime = 0;
bool clawDisabled = false;
int clawDirection = 0;
double preAtargetpos = 0;
bool shotFired = false;

bool releasing = false;
uint32_t releaseStart = 0;

void setClaw(int power) {
    if (power != 0 && power != clawDirection) {
        clawMoveStart = pros::millis();
        clawDirection = power;
    }

    if (!clawDisabled) {
        claw.move(power);
    }
}

// lemlib::PID liftPID(0.1, 0, 0.85, 3000, true);
lemlib::PID clawPID(0.02, 0, 0.06, 3000, true);
// Light objects
lemlib::PID liftPIDLight(0.10, 0, 0.85, 3000, true);

// Heavy objects
lemlib::PID liftPIDHeavy(0.15, 0, 1.10, 3000, true);

// Active PID
lemlib::PID* liftPID = &liftPIDLight;

//helpers

void driveLiftTo(double target, uint32_t timeoutMs = 2000) {
    liftPID->reset();
    uint32_t start = pros::millis();

    while (pros::millis() - start < timeoutMs) {
        double current = liftrot.get_position();
        double error = target - current;

        double output = liftPID->update(error);
        output = std::clamp(output, -127.0, 127.0);

        lift.move(output);

        if (std::fabs(error) < 50) break;
        pros::delay(10);
    }
    lift.brake();
    liftPID->reset();
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

void score() {
    clawtarget = clawstart + 33000;
    pros::delay(700);
    setClaw(127);
    pros::delay(100);
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

void liftone(int amount) {
    double maxpos = startpos + maxincrements * increment;
    double target = startpos + amount;
    target = std::clamp(target, startpos, maxpos);

    driveLiftTo(target);
    pros::delay(500);
    setClaw(127); // spin claw outward
    pros::delay(500);
    driveClawRotatorTo(clawstart + 51000); // rotate claw up to max
}

//macro 2: claw rotator all the way down + arm to startpos

void resetLiftAndClaw() {
    liftPID->reset();
    clawPID.reset();
    bool liftDone = false;
    bool clawDone = false;
    uint32_t start = pros::millis();

    while (pros::millis() - start < 2000 && !(liftDone && clawDone)) {
        if (!liftDone) {
            double liftError = startpos - liftrot.get_position();
            double liftOutput = liftPID->update(liftError);
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
    liftPID->reset();
    clawPID.reset();
}
const double CLAW_MIN_RPM = 50;          // normal running threshold
const double CLAW_TEST_MIN_RPM = 15;      // recovery threshold
const uint32_t CLAW_SPINUP_TIME = 500;

const int CLAW_TEST_POWER = 120;
bool resumeIntakeAfterJam = false;
void clawAntiJamTask(void*) {
    while (true) {

        if (!clawDisabled) {

            if (clawDirection != 0 &&
                pros::millis() - clawMoveStart > CLAW_SPINUP_TIME) {

                if (std::fabs(claw.get_actual_velocity()) < CLAW_MIN_RPM) {
                    resumeIntakeAfterJam = intakeToggle;
                    clawDisabled = true;
                    intakeToggle = false;
                    // Immediately begin slow recovery
                    claw.move((clawDirection > 0) ?
                        CLAW_TEST_POWER :
                        -CLAW_TEST_POWER);
                }
            }

        } else {

            // Keep slowly pushing the whole time
            claw.move((clawDirection > 0) ?
                CLAW_TEST_POWER :
                -CLAW_TEST_POWER);

            // As soon as it starts moving again...
            if (std::fabs(claw.get_actual_velocity()) >= CLAW_TEST_MIN_RPM) {

                clawDisabled = false;
                    if (resumeIntakeAfterJam) {
                        intakeToggle = true;
                    }
                // Resume normal speed
                resumeIntakeAfterJam = false;
                claw.move(clawDirection);
                clawMoveStart = pros::millis();
            }
        }

        pros::delay(10);
    }
}

void mechanismTask(void*) {
    while (true) {

        currentpos = liftrot.get_position();
        clawcurrent = clawrot.get_position();

        // Lift
        {
            
                double error = targetpos - currentpos;
                double output = liftPID->update(error);
                output = std::clamp(output, -127.0, 127.0);
                lift.move(output);
            
        }

        // Claw rotator
        {
            double error = clawtarget - clawcurrent;
            double output = clawPID.update(error);
            output = std::clamp(output, -127.0, 127.0);
            clawrotator.move(output);
        }

        pros::delay(10);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    startpos = liftrot.get_position();
    //lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawrotator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawstart = clawrot.get_position();
    clawtarget = clawstart;
    targetpos = startpos;
    pros::Task antiJamTask(clawAntiJamTask);
    pros::Task mechTask(mechanismTask);
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


void scoreA(uint32_t holdTimeMs = 300) {
    double maxpos = startpos + maxincrements * increment;
    double preTarget = targetpos;

    // "press" phase
    clawtarget = clawstart + 33000;
    targetpos = std::clamp(targetpos - (increment / 2), startpos, maxpos);
    pros::delay(500);
    setClaw(127);
    pros::delay(holdTimeMs);   // how long the shot spins out, like holding A

    // "release" phase
    targetpos = std::clamp(preTarget + 1000, startpos, maxpos);
    clawtarget = clawstart + 60000;
    setClaw(-127);
    pros::delay(400);
    setClaw(127);
    pros::delay(300);
    setClaw(-127);
}

void getstack() {
    targetpos = startpos;
    pros::delay(200);             // lower the lift
    clawtarget=clawstart+3900;
    pros::delay(600);
    clawtarget = clawstart + 35000;   // come back up, slightly lower than A
}

void getstackfromwall() {
    // Move claw slightly down from the wall position
    clawtarget = clawstart + 35000;
    pros::delay(1000);

    // Now do the normal getstack motion
    targetpos = startpos;

    // Rotate claw farther down
    clawtarget = clawstart + 5000;
    pros::delay(500);

    // Rotate back up slightly below the A position
    clawtarget = clawstart + 35000;
}
void elim() {
    chassis.setPose(-40.625, -62.75, 180);
    liftPID = &liftPIDLight;
    liftPID->reset();
    chassis.moveToPoint(-30.079, -52.736, 900, {.forwards = false});
    chassis.waitUntil(1.75);
    clawtarget = clawstart + 33000;
    pros::delay(400);
    setClaw(127);
    pros::delay(200);
    targetpos = startpos + 500;
    clawtarget = clawstart + 56000;
    chassis.moveToPoint(-36.912, -60.486, 650);
    chassis.turnToPoint(-40.15, -56.5, 750, {.forwards = false});
    setClaw(-127);
    chassis.moveToPoint(-40.4, -56.5, 1000, {.forwards = false, .maxSpeed = 90});
    //chassis.turnToPoint(-38.233, -56.184, 1000, {.forwards = false});
    //chassis.moveToPoint(-38.233, -56.184, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(1.5);
    getstack();
    liftPID = &liftPIDHeavy;
    liftPID->reset();
    targetpos = startpos + 3100;
    //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //chassis.moveToPoint(-32.912, -66.486, 1000, {.earlyExitRange = 5});
    // chassis.swingToHeading(270, DriveSide::RIGHT, 1000, {.earlyExitRange = 5});
    chassis.moveToPoint(-48.495, -44.897, 800, {.forwards = false});
    clawtarget = clawstart + 33000;
    chassis.turnToPoint(-23, -46, 900, {.forwards = false});
    chassis.moveToPoint(-23, -46., 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(275);
    setClaw(127);
    //DSR
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    DsrMain.updateBotPose(&front_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    pros::delay(100);
    chassis.moveToPoint(-39.5, -46, 750);
    chassis.turnToPoint(-28.481, -31.5, 800, {.forwards = false});
    setClaw(-127);
    targetpos = startpos;
    clawtarget = clawstart + 54000;
    chassis.moveToPoint(-28.481, -31.5, 1100, {.forwards = false});
    chassis.waitUntil(11);
    getstack();
    chassis.turnToPoint(18.323, -45.417, 700, {.forwards = false});
    targetpos = startpos + 4200;
    clawtarget = clawstart + 33000;
    chassis.moveToPoint(18.323, -45.417, 1300, {.forwards = false});
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    pros::delay(150);
    setClaw(127);
    //DSR
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    pros::delay(275);
    chassis.moveToPoint(1.5, -40, 700);
    chassis.turnToPoint(1.5, -71, 600, {.forwards = false});
    clawtarget = clawstart;
    targetpos = startpos;
    chassis.moveToPoint(1.5, -71, 800, {.forwards = false, .minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(1.5, -52, 450);
    chassis.waitUntilDone();
    chassis.moveToPoint(1.5, -71, 800, {.forwards = false, .minSpeed = 80});
}
void sawp() {
    chassis.setPose(-0.217, -62.801, 180);
    setClaw(-127);
    chassis.moveToPoint(-14.581, -38.081, 800, {.forwards = false});
    chassis.turnToPoint(-23.072, -46.925, 550, {.forwards = false}); 
    clawtarget = clawstart + 51000;
    pros::delay(600);
    chassis.moveToPoint(-20.256, -47.166, 800, {.forwards = false}); // -20.056, -42.904
    chassis.waitUntil(1);
    clawtarget = clawstart + 33000;
    pros::delay(400);
    setClaw(127);
    pros::delay(200);
    targetpos = startpos;
    chassis.moveToPoint(-13.718, -38.006, 540);
    clawtarget = clawstart + 51000;
    chassis.turnToPoint(-18.866, -34.603, 600, {.forwards = false});
    setClaw(-127);
    chassis.moveToPoint(-18.866, -34.603, 400, {.forwards = false});
    chassis.waitUntil(0.2);
    getstack();
    targetpos = startpos + 6800;
    chassis.turnToPoint(-0.5, -63.239, 400);
    chassis.moveToPoint(-0.5, -63.239, 850);
    chassis.waitUntilDone();
    liftPID = &liftPIDHeavy;
    liftPID->reset();
    chassis.turnToHeading(0, 600);
    chassis.moveToPoint(-0.5, -70.773, 630, {.forwards = false, .minSpeed = 120});
    pros::delay(50);
    chassis.moveToPoint(-0.5, -56.603, 620);
    chassis.turnToPoint(18.819, -57.503, 650, {.forwards = false});
    targetpos = startpos + 4800;
    chassis.moveToPoint(20.819, -57.503, 950, {.forwards = false});
    chassis.waitUntil(15);
    clawtarget = clawstart + 34000;
    pros::delay(250);
    setClaw(127);
    pros::delay(300);
    //DSR
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // chassis.turnToPoint(1, -47.003, 250);
    // chassis.moveToPoint(1, -47.003, 660);
    // clawtarget = clawstart + 34000;
    // setClaw(-127);
    // pros::delay(50);
    // chassis.turnToPoint(1, -69.773, 600, {.forwards = false});
    // pros::delay(50);
    // targetpos = startpos + 3500;
    // chassis.moveToPoint(1, -69.773, 900, {.forwards = false, .minSpeed = 120});
    // chassis.waitUntilDone();
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // pros::delay(50);
    // chassis.moveToPoint(1, -59, 900);
    // chassis.turnToPoint(-40, -59, 650, {.forwards = false});
    // chassis.moveToPoint(-40, -59, 1100, {.forwards = false});
    // chassis.waitUntilDone();
    // pros::delay(50);
    // targetpos = startpos + 2100;
    // clawtarget = clawstart + 51000;
    // chassis.turnToHeading(163, 1000);
    // chassis.waitUntilDone();
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // DsrMain.updateBotPose(&right_dsr);   // Distance reset on the left sensor
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // pros::delay(50);
    // chassis.moveToPose(-48.632, -53.185, 147.5, 900, {.forwards = false});
    // chassis.waitUntil(2.5);
    // ///GRAB THE PIN///
    // getstack();
    // chassis.turnToPoint(-27.556, -47.966, 600, {.forwards = false}); 
    // clawtarget = clawstart + 510000;
    // targetpos = startpos + 3000;
    // pros::delay(150);
    // chassis.moveToPoint(-27.556, -47.966, 600, {.forwards = false}); // -20.056, -42.904
    // chassis.waitUntil(7.5);
    // clawtarget = clawstart + 33500;
    // pros::delay(150);
    // setClaw(127);
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    // DsrMain.updateBotPose(&front_dsr);   // Distance reset on the left sensor
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
}

void skills98() {

    chassis.setPose(-69.104, 0.291, 90);
    liftPID = &liftPIDLight;

    //targetpos = startpos + 5100;
    setClaw(-127);
    chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
    chassis.moveToPoint(-72, 0.291,800,{.forwards=false,.minSpeed=120});
    chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
    chassis.moveToPoint(-72, 0.291,800,{.forwards=false, .minSpeed=120});
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.swingToHeading(-10,DriveSide::LEFT,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
    intake.move(127);
    targetpos = startpos;
    
    chassis.waitUntilDone();
    chassis.moveToPoint(-60,-14,1000,{.forwards=false,.maxSpeed=90});
    pros::delay(150);
    clawtarget = clawstart + 33000;
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    pros::delay(200);
    setClaw(127);
    pros::delay(200);
    targetpos = startpos;
    chassis.waitUntilDone();

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.updateBotPose(&front_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    pros::delay(100);
    clawtarget=clawstart+50000;
    setClaw(-127);

    chassis.moveToPoint(-47.067,-6.154,800,{.maxSpeed=80});
    chassis.turnToHeading(-40,700,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-28.559,-19.332,800,{.forwards=false,.maxSpeed=70});
    lift.move(-30);
    targetpos=startpos;
    chassis.waitUntilDone();
    
    
    pros::Task liftTask1([]() {
    while (true) {
        lift.move(-100);
        pros::delay(10);
    }
    });

    targetpos = startpos;
    chassis.waitUntilDone();
    pros::delay(150);

    targetpos = startpos;
    clawtarget=clawstart+3900;
    pros::delay(600);
    clawtarget = clawstart + 35000;   // come back up, slightly lower than A

    liftTask1.remove();
    lift.move(0);



    liftPID = &liftPIDHeavy;
    lift.move(0);
    chassis.turnToHeading(-287,800,{.maxSpeed=80});
    clawtarget=clawstart+30000;
    targetpos = startpos+4000;
    chassis.moveToPoint(-46.859,-24.032,700,{.forwards=false,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    
    pros::delay(100);
    setClaw(127);
    // scoreA();

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&right_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    pros::delay(300);
    liftPID = &liftPIDLight;
    clawtarget=clawstart+40000;
    chassis.moveToPoint(-30.332,-23.513,800,{.maxSpeed=110});
    chassis.waitUntilDone();
    chassis.moveToPoint(-29.604,-30.373,900,{.maxSpeed=100});
    chassis.waitUntilDone();
    targetpos=startpos;
    chassis.turnToHeading(42,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=85});
    clawtarget=clawstart+50000;
    
    chassis.waitUntilDone();
    chassis.moveToPoint(-41.375,-42.909,800,{.forwards=false,.maxSpeed=85});
    setClaw(-127);
    chassis.waitUntilDone();



    pros::Task liftTask2([]() {
    while (true) {
        lift.move(-127);
        pros::delay(10);
    }
    });

    targetpos = startpos;
    chassis.waitUntilDone();

    getstack();

    liftTask2.remove();
    lift.move(0);
    



    chassis.moveToPoint(-51.375,-51.509,800,{.forwards=false,.maxSpeed=95});
    liftPID = &liftPIDHeavy;

    
    chassis.turnToHeading(153,700,{.maxSpeed=100});
    clawtarget=clawstart+30000;
    targetpos = startpos+6800;
    chassis.waitUntilDone();
    chassis.moveToPoint(-53.547,-30.176,900,{.forwards=false,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.arcade(-70, 0);
    
    pros::delay(100);
    // setClaw(127);
    scoreA();

    clawtarget=clawstart+600000;
    // targetpos=startpos+6500;





//////// MATCHHHHHH LOADDINGGGGGG






    chassis.moveToPoint(-49.77,-59.854,900,{.maxSpeed=100});
    pros::delay(400);
    targetpos=startpos;
    clawtarget=clawstart;
    chassis.waitUntilDone();
    chassis.turnToHeading(270,750,{.maxSpeed=100});
    chassis.waitUntilDone();

    chassis.moveToPoint(-72.272,-59.466,700,{.maxSpeed=110,.minSpeed=20});
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(800);

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    setClaw(-127);
    liftPID = &liftPIDHeavy;
    //first stack
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    pros::delay(200);
    clawtarget = clawstart + 33000;
    chassis.waitUntilDone();
    chassis.arcade(-80, 0);
    pros::delay(300);
    chassis.arcade(0,0);
    setClaw(127);
    pros::delay(200);
    targetpos = startpos;



    chassis.moveToPoint(-63.917,-59.8,800,{.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.turnToHeading(270,500,{.maxSpeed=90});
    setClaw(-127);
    clawtarget=clawstart;
    
    chassis.moveToPoint(-72.272,-58.6,900,{.maxSpeed=100});
    chassis.waitUntilDone();
    chassis.arcade(90,0);
    pros::delay(1000);

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    // //SECOND STACK
    chassis.moveToPoint(-38.543,-49.345,800,{.forwards=false,.maxSpeed=80});
    pros::delay(200);
    clawtarget=clawstart+33000;
    targetpos = startpos+3800;
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    
    chassis.waitUntilDone();
    chassis.arcade(-100, 0);
    
    pros::delay(150);
    chassis.arcade(0,0);
    setClaw(127);
    pros::delay(200);
    clawtarget=clawstart+60000;

    chassis.moveToPoint(-63.917,-59.4,1000,{.maxSpeed=90});
    setClaw(-127);
    chassis.waitUntilDone();
    chassis.turnToHeading(270,500,{.maxSpeed=90});
    clawtarget=clawstart;
    chassis.moveToPoint(-72.272,-58.9,900,{.maxSpeed=80});
    setClaw(-127);
    targetpos = startpos;
    clawtarget = clawstart;
    chassis.waitUntilDone();
    chassis.arcade(90,0);
    pros::delay(1000);


    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    

    //THIRD STACK
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    clawtarget=clawstart+28000;
    targetpos = startpos+6500;
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-80, 0);

    pros::delay(300);
    chassis.arcade(0,0);

    setClaw(127);
    pros::delay(200);

    clawtarget=clawstart+60000;


    chassis.moveToPoint(-63.917,-58.9,800,{.maxSpeed=90});
    setClaw(-127);
    chassis.turnToHeading(270,500,{.maxSpeed=100});
    clawtarget=clawstart;
    targetpos=startpos;
    chassis.moveToPoint(-72.272,-57.9,900,{.maxSpeed=100});
    
    setClaw(-127);
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(1000);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    
    //FOURTH STACK
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    clawtarget=clawstart+30000;
    targetpos = startpos+9000;
    chassis.moveToPoint(-25.03,-47.232,800,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-70, 0);

    pros::delay(300);
    chassis.arcade(0,0);

    pros::delay(400);
    setClaw(127);
    pros::delay(200);

    clawtarget=clawstart+60000;

    chassis.moveToPoint(-63.917,-58,900,{.maxSpeed=100});
    chassis.turnToHeading(270,400,{.maxSpeed=100});
    setClaw(-127);
    clawtarget=clawstart;
    targetpos=startpos;
    chassis.moveToPoint(-72.272,-58,600,{.maxSpeed=100});
    
    setClaw(-127);
    
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(1000);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    
    //FIFTH STACK
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    clawtarget=clawstart+33000;
    targetpos = startpos+11300;
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-70, 0);

    pros::delay(300);
    chassis.arcade(0,0);

    pros::delay(400);
    setClaw(127);
    pros::delay(200);

    targetpos=startpos+11000;
    clawtarget=clawstart+60000;


    // //OTHER GOAL
    

    chassis.moveToPoint(-63.917,-58.4,1000,{.maxSpeed=90});
    setClaw(-127);
    clawtarget=clawstart;
    targetpos=startpos;
    chassis.turnToHeading(270,400,{.maxSpeed=100});
    
    chassis.moveToPoint(-72.272,-58.4,900,{.maxSpeed=100});
    setClaw(-127);
    chassis.waitUntilDone();
    chassis.arcade(100, 0);
    pros::delay(1000);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    
    //SIXTH STACK
    chassis.moveToPoint(-58.314,-48.148,800,{.forwards=false,.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.turnToHeading(180,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
    clawtarget=clawstart+30000;
    targetpos = startpos+9000;
    chassis.waitUntilDone();
    chassis.moveToPoint(-57.483,-20.448,1200,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-60, 0);

    pros::delay(300);
    chassis.arcade(0,0);
    pros::delay(600);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&right_dsr);
    DsrMain.updateBotPose(&front_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose



    setClaw(127);
    pros::delay(300);



    ///PARKINGGGGGGGGGGG
    

    chassis.moveToPoint(-53.154, -31.353, 500, {.maxSpeed=80, .earlyExitRange = 4});
    chassis.turnToHeading(80,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90, .earlyExitRange = 4});
    pros::delay(400);
    targetpos = startpos;
    clawtarget = clawstart;
    chassis.moveToPoint(6, chassis.getPose().y, 1000, {.maxSpeed=100, .earlyExitRange = 5});
    //chassis.waitUntilDone();
    chassis.turnToHeading(0, 800, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -75, 1300, {.forwards = false, .minSpeed = 120});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -53, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -75, 1000, {.forwards = false, .minSpeed = 120});
    chassis.waitUntilDone();
    chassis.moveToPoint(0,-8,3000,{.minSpeed=100});
}












void skills88() {

    chassis.setPose(-69.104, 0.291, 90);
    liftPID = &liftPIDLight;

    //targetpos = startpos + 5100;
    setClaw(-127);
    chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
    chassis.moveToPoint(-72, 0.291,800,{.forwards=false,.minSpeed=120});
    chassis.moveToPoint(-63,0.291,700,{.maxSpeed=110});
    chassis.moveToPoint(-72, 0.291,800,{.forwards=false, .minSpeed=120});
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.swingToHeading(-10,DriveSide::LEFT,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
    intake.move(127);
    targetpos = startpos;
    
    chassis.waitUntilDone();
    chassis.moveToPoint(-60,-14,1000,{.forwards=false,.maxSpeed=90});
    pros::delay(150);
    clawtarget = clawstart + 33000;
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    pros::delay(200);
    setClaw(127);
    pros::delay(200);
    targetpos = startpos;
    chassis.waitUntilDone();

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.updateBotPose(&front_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    pros::delay(100);
    clawtarget=clawstart+50000;
    setClaw(-127);

    chassis.moveToPoint(-47.067,-6.154,800,{.maxSpeed=80});
    chassis.turnToHeading(-40,700,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-28.559,-19.332,800,{.forwards=false,.maxSpeed=70});
    lift.move(-30);
    targetpos=startpos;
    chassis.waitUntilDone();
    
    
    pros::Task liftTask1([]() {
    while (true) {
        lift.move(-100);
        pros::delay(10);
    }
    });

    targetpos = startpos;
    chassis.waitUntilDone();
    pros::delay(150);

    targetpos = startpos;
    clawtarget=clawstart+3900;
    pros::delay(600);
    clawtarget = clawstart + 35000;   // come back up, slightly lower than A

    liftTask1.remove();
    lift.move(0);



    liftPID = &liftPIDHeavy;
    lift.move(0);
    chassis.turnToHeading(-287,800,{.maxSpeed=80});
    clawtarget=clawstart+28000;
    targetpos = startpos+4000;
    chassis.moveToPoint(-46.859,-24.032,700,{.forwards=false,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    
    pros::delay(100);
    // setClaw(127);
    scoreA();

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&right_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    liftPID = &liftPIDLight;
    clawtarget=clawstart+40000;
    chassis.moveToPoint(-30.332,-23.513,800,{.maxSpeed=110});
    chassis.waitUntilDone();
    chassis.moveToPoint(-29.604,-30.373,900,{.maxSpeed=100});
    chassis.waitUntilDone();
    targetpos=startpos;
    chassis.turnToHeading(42,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=85});
    clawtarget=clawstart+50000;
    
    chassis.waitUntilDone();
    chassis.moveToPoint(-42.375,-43.909,800,{.forwards=false,.maxSpeed=85});
    setClaw(-127);
    chassis.waitUntilDone();



    pros::Task liftTask2([]() {
    while (true) {
        lift.move(-127);
        pros::delay(10);
    }
    });

    targetpos = startpos;
    chassis.waitUntilDone();

    getstack();

    liftTask2.remove();
    lift.move(0);
    



    chassis.moveToPoint(-51.375,-51.509,800,{.forwards=false,.maxSpeed=95});
    liftPID = &liftPIDHeavy;

    
    chassis.turnToHeading(153,700,{.maxSpeed=100});
    clawtarget=clawstart+30000;
    targetpos = startpos+6800;
    chassis.waitUntilDone();
    chassis.moveToPoint(-53.547,-30.176,900,{.forwards=false,.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.arcade(-70, 0);
    
    pros::delay(100);
    // setClaw(127);
    scoreA();

    clawtarget=clawstart+600000;
    // targetpos=startpos+6500;





//////// MATCHHHHHH LOADDINGGGGGG






    chassis.moveToPoint(-49.77,-59.854,900,{.maxSpeed=100});
    pros::delay(400);
    targetpos=startpos;
    clawtarget=clawstart;
    chassis.waitUntilDone();
    chassis.turnToHeading(270,750,{.maxSpeed=100});
    chassis.waitUntilDone();

    chassis.moveToPoint(-72.272,-59.466,700,{.maxSpeed=110,.minSpeed=20});
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(800);

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    setClaw(-127);
    liftPID = &liftPIDHeavy;
    //first stack
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    pros::delay(200);
    clawtarget = clawstart + 33000;
    chassis.waitUntilDone();
    chassis.arcade(-80, 0);
    pros::delay(300);
    chassis.arcade(0,0);
    setClaw(127);
    //scoreA();
    pros::delay(200);
    targetpos = startpos;



    chassis.moveToPoint(-63.917,-59.8,800,{.maxSpeed=90});
    chassis.waitUntilDone();
    chassis.turnToHeading(270,500,{.maxSpeed=90});
    setClaw(-127);
    clawtarget=clawstart;
    
    chassis.moveToPoint(-72.272,-58.6,900,{.maxSpeed=100});
    chassis.waitUntilDone();
    chassis.arcade(90,0);
    pros::delay(1000);

    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    // //SECOND STACK
    chassis.moveToPoint(-38.543,-49.345,800,{.forwards=false,.maxSpeed=80});
    pros::delay(200);
    clawtarget=clawstart+33000;
    targetpos = startpos+3800;
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    
    chassis.waitUntilDone();
    chassis.arcade(-100, 0);
    
    pros::delay(150);
    chassis.arcade(0,0);
    // setClaw(127);
    scoreA();
    pros::delay(200);
    clawtarget=clawstart+60000;

    chassis.moveToPoint(-63.917,-59.4,1000,{.maxSpeed=90});
    setClaw(-127);
    chassis.waitUntilDone();
    chassis.turnToHeading(270,500,{.maxSpeed=90});
    clawtarget=clawstart;
    chassis.moveToPoint(-72.272,-58.9,900,{.maxSpeed=80});
    setClaw(-127);
    targetpos = startpos;
    clawtarget = clawstart;
    chassis.waitUntilDone();
    chassis.arcade(90,0);
    pros::delay(1000);


    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    

    //THIRD STACK
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    clawtarget=clawstart+28000;
    targetpos = startpos+6500;
    chassis.moveToPoint(-25.03,-47.732,800,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-80, 0);

    pros::delay(300);
    chassis.arcade(0,0);

    // setClaw(127);
    scoreA();
    pros::delay(100);

    clawtarget=clawstart+60000;


    chassis.moveToPoint(-63.917,-58.9,800,{.maxSpeed=90});
    setClaw(-127);
    chassis.turnToHeading(270,500,{.maxSpeed=100});
    clawtarget=clawstart;
    targetpos=startpos;
    chassis.moveToPoint(-72.272,-57.9,900,{.maxSpeed=100});
    
    setClaw(-127);
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(1000);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose

    
    //FOURTH STACK
    chassis.moveToPoint(-38.543,-48.345,800,{.forwards=false,.maxSpeed=80});
    clawtarget=clawstart+30000;
    targetpos = startpos+9000;
    chassis.moveToPoint(-25.03,-47.232,800,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-70, 0);

    pros::delay(300);
    chassis.arcade(0,0);

    pros::delay(400);
    // setClaw(127);
    scoreA();
    pros::delay(100);

    clawtarget=clawstart+60000;

    chassis.moveToPoint(-63.917,-58,900,{.maxSpeed=100});
    chassis.turnToHeading(270,400,{.maxSpeed=100});
    setClaw(-127);
    clawtarget=clawstart;
    targetpos=startpos;
    chassis.moveToPoint(-72.272,-58,600,{.maxSpeed=100});
    
    setClaw(-127);
    
    chassis.waitUntilDone();
    chassis.arcade(100,0);
    pros::delay(1000);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose



    
    //SIXTH STACK
    chassis.moveToPoint(-58.314,-48.148,800,{.forwards=false,.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.turnToHeading(180,800,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
    clawtarget=clawstart+30000;
    targetpos = startpos+9000;
    chassis.waitUntilDone();
    chassis.moveToPoint(-57.483,-20.448,1200,{.forwards=false,.maxSpeed=70});
    chassis.waitUntilDone();
    chassis.arcade(-60, 0);

    pros::delay(300);
    chassis.arcade(0,0);
    pros::delay(400);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&right_dsr);
    DsrMain.updateBotPose(&front_dsr);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose



    // setClaw(127);
    scoreA();
    pros::delay(100);



    ///PARKINGGGGGGGGGGG
    

    chassis.moveToPoint(-53.154, -31.353, 500, {.maxSpeed=80});
    chassis.turnToHeading(80,900,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90});
    pros::delay(400);
    targetpos = startpos;
    clawtarget = clawstart;
    chassis.moveToPoint(6, chassis.getPose().y, 1000, {.maxSpeed=100, .earlyExitRange = 5});
    //chassis.waitUntilDone();
    chassis.turnToHeading(0, 800, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -75, 1300, {.forwards = false, .minSpeed = 120});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -53, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x, -75, 1000, {.forwards = false, .minSpeed = 120});
    chassis.waitUntilDone();
    chassis.moveToPoint(0,-8,3000,{.minSpeed=100});
}








void ethan() {
    chassis.setPose(-62.5, -0.3125, 90);
    setClaw(-127);
    intake.move(127);
    chassis.moveToPoint(-26.98, -0.3125, 850, {.earlyExitRange = 5});
    chassis.waitUntilDone();
    chassis.moveToPoint(-17.98, -0.3125, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToPoint(-43.612, 19.168, 620, {.forwards = false});
    chassis.moveToPoint(-43.612, 19.168, 1000, {.forwards = false});
    pros::delay(150);
    targetpos = startpos + 5000;
    clawtarget = clawstart + 34400;
    chassis.waitUntilDone();
    chassis.arcade(-127, 0);
    pros::delay(125);
    setClaw(127);
    pros::delay(200);
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    // DsrMain.updateBotPose(&right_dsr);   // Distance reset on the left sensor
    // DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    clawtarget = clawstart + 51000;
    chassis.moveToPoint(-35.982, 8.652, 600);
    setClaw(-127);
    intake.move(127);
    chassis.turnToPoint(-25.246, 17.263, 630);
    clawtarget = clawstart;
    targetpos = startpos;
    chassis.moveToPoint(-25.246, 17.263, 800);
    pros::delay(150);
    chassis.turnToPoint(-47.193, -17.083, 500, {.forwards = false});
    chassis.moveToPoint(-47.193, -17.083, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clawtarget = clawstart + 35000;
    chassis.arcade(-127, 0);
    pros::delay(125);
    setClaw(127);
    pros::delay(50);
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    pros::delay(250);
    chassis.moveToPoint(-37.635, -11.417, 900);
    setClaw(-127);
    targetpos = startpos;
    clawtarget = clawstart + 51000;
    chassis.turnToPoint(-31.96, -16.439, 900, {.forwards = false});
    chassis.waitUntilDone();
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    DsrMain.updateBotPose(&left_dsr);   // Distance reset on the left sensor
    DsrMain.setDsrPose(chassis.getPose());  // Reset dsr Pose to Lemlib Pose
    chassis.moveToPoint(-31.96, -16.439, 430, {.forwards = false});
    chassis.waitUntilDone();
    getstack();
    pros::delay(75);
    chassis.turnToPoint(-43.823, -22.251, 650, {.forwards = false});
    targetpos = startpos + 3800;
    chassis.moveToPoint(-43.853, -22.251, 760, {.forwards = false});
    clawtarget = clawstart + 34400;
    chassis.waitUntilDone();
    setClaw(127);
    pros::delay(250);
    chassis.swingToHeading(0, DriveSide::LEFT, 700, {.earlyExitRange = 12});
    chassis.moveToPoint(-38, 0, 800, {.earlyExitRange = 7});
    targetpos = startpos + 1000;
    clawtarget = clawstart;
    chassis.turnToPoint(-72, 0, 600, {.forwards = false});
    chassis.moveToPoint(-73, 0, 1100, {.forwards = false, .minSpeed = 120});
    chassis.waitUntil(20.5);
    chassis.tank(-127, -127);
    targetpos = startpos + 8000;
    clawtarget = clawstart + 51000;
}

void autonomous() {
    skills88();
}


void opcontrol() {
    liftPID = &liftPIDLight;
    liftPID->reset();
    clawMoveStart = pros::millis();
    currentpos = startpos;
    //targetpos = startpos;
    clawcurrent = clawstart;
    //clawtarget = clawstart;
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
            targetpos -= (increment / 2);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && targetpos < maxpos) {
            if (targetpos == startpos) {
                targetpos += 500;
            } else {
                targetpos += increment;
            }

            if (clawtarget < clawstart + 51000) {
                clawtarget = clawstart + 51000;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            targetpos = startpos;
            clawtarget = clawstart;
        }

        if (!clawDisabled) {
            setClaw(-127);
        }

        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     wasHoldingA = true;

        //     if (!timerStarted) {
        //         timerStarted = true;
        //         startTime = pros::millis();
        //         clawtarget = clawstart + 33000;
        //     }

        //     if (pros::millis() - startTime >= 500) {
        //         setClaw(127);
        //     }
        // }
        // else if (wasHoldingA) {
        //     wasHoldingA = false;
        //     timerStarted = false;
        //     clawtarget = clawstart + 51000;
        //     setClaw(-127);
        // }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    wasHoldingA = true;

    if (!timerStarted) {
        timerStarted = true;
        startTime = pros::millis();
        preAtargetpos = targetpos;                 // remember height before A
        clawtarget = clawstart + 33000;             // rotate claw to scoring position
        targetpos = std::clamp(targetpos - (increment / 2), startpos, maxpos); // drop half increment
        shotFired = false;
        releasing = false;
    }

    if (pros::millis() - startTime >= 500) {
        setClaw(127);                               // spin outward, re-asserted every loop
        shotFired = true;
    }
}
else if (wasHoldingA) {
    wasHoldingA = false;
    timerStarted = false;
    shotFired = false;

    if (!releasing) {
    releasing = true;
    releaseStart = pros::millis();
    targetpos = std::clamp(preAtargetpos + 1000, startpos, maxpos); // go a bit above original height
    clawtarget = clawstart + 51000;              // rotate claw back up
    setClaw(-127);                               // hold claw inward while lift rises
}

    if (releasing && pros::millis() - releaseStart >= 400) {
        setClaw(127);                                // spin outward again once up
    }

    if (releasing && pros::millis() - releaseStart >= 700) {
        setClaw(-127);                                // finally settle back to normal inward
        releasing = false;
    }
}


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeToggle = !intakeToggle;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        }
        else if (intakeToggle) {
            intake.move(127);
        }
        else {
            intake.move(0);
        }
        pros::delay(10);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {

        pros::Task liftTask([]() {
        while (true) {
            lift.move(-100);
            pros::delay(10);
        }
        });

        getstack();

        liftTask.remove();  // Stop the task
        lift.move(0);





            // lift.move(-60);
            // getstack();
            // lift.move(0);
        }

        

    }
}
