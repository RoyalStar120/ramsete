#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "primitives.hpp"
#include "splinepath.hpp"
#include "trajectory.hpp"
#include "ramsete.hpp"
#include "dsr.hpp"

const double TRACK_WIDTH = 10.75;
const double MAX_VEL     = 59;
const double MAX_ACCEL   = 60;  
const double MU          = 0.5;
const double B           = 1.7; 
const double ZETA        = 0.9;





// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-4, -5, 6},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 2, -3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)




// Inertial Sensor on port 10
pros::Imu imu(17);
pros::adi::Pneumatics clawrotate('C', true);
pros::adi::Pneumatics clawopen('D', true);

pros::Motor lift(15);
pros::Motor intake(-20);
pros::Rotation liftrot(-12);
pros::Motor claw(13);
pros::Motor clawrotator(14);
pros::Rotation clawrot(16);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(7);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            28, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

SplinePath splinePath(50);
Trajectory trajectory(MAX_VEL, MAX_ACCEL, TRACK_WIDTH, MU);
RAMSETE ramsete(B, ZETA, TRACK_WIDTH, MAX_VEL);

void followPath(std::vector<WayPoint> waypoints) {
	auto splinePoints=splinePath.generate(waypoints);
	auto trajectoryPoints=trajectory.generate(splinePoints);
	ramsete.follow(trajectoryPoints, chassis);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
double startpos;
double clawstart;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    startpos = liftrot.get_position(); //centidegrees
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawrotator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawstart = clawrot.get_position();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "liftrot: %f", liftrot.get_position());
            pros::lcd::print(4, "clawrot: %f", clawrot.get_position());
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
std::vector<WayPoint> myPath = {
    {0,  0,  90},   // start facing up
    {24, 24, 45},   // middle point, angled diagonal
    {48, 0,  0}     // end facing right
};



void autonomous() {
    chassis.setPose(0, 0, 0);
    followPath(myPath);
}



bool movingarm = false;
bool movingclaw = false;
// double targetpos = 14355;
// const double max = 36000;
// const double min = 14355;
const double increment = 3000; //centidegfrees
const double maxincrements = 5; // CHANGE THIS SHIT
double targetpos;
double currentpos;
double clawcurrent;
double clawtarget;
lemlib::PID liftPID(0.1, 0, 0.01, 3000, true);
lemlib::PID clawPID(0.02, 0, 0.06, 3000, true);
void opcontrol() {
    // controller
    // loop to continuously update motors
	bool intakeToggle = false;
    currentpos = startpos;
    targetpos = startpos;
    clawcurrent = clawstart;
    clawtarget = clawstart;
    const double maxpos = startpos + maxincrements * increment;

    while (true) {
        currentpos = liftrot.get_position(); //centidegrees
        clawcurrent = clawrot.get_position();
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources

        //rot sensor used to create macros

        ///ALL MACROS ARE HERE (DR4B)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && targetpos > startpos) {
            targetpos -= increment;
            // if (targetpos > max) targetpos = max;
            movingarm = true;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && targetpos < maxpos) {
            targetpos += increment;
            movingarm = true;
            if (clawtarget < (clawstart + 100)) { //17946
                clawtarget = clawstart + 100;
                movingclaw = true;
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            targetpos = startpos;
            movingarm = true;
            clawtarget = clawstart;
            movingclaw = true;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && clawtarget > (clawstart + 6546)) {
            if (clawtarget > (clawstart + 33000)) {
                clawtarget = clawstart + 33000;
                movingclaw = true;
            }
            claw.move(127);
        } else {
            claw.move(-127);
        }
/*
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            clawrotator.move(127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            clawrotator.move(-127);
        } else {
            clawrotator.move(0);
        }
*/
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
      //lift pid
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
		} else if (intakeToggle) {
			intake.move(-127);
		}
		else {
			intake.move(0);
		}
        
		// if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		// 	lift.move(-127);
        //     		} 
		// else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		// 	lift.move(127);
        //     		} else {
		// 	lift.move(0);
		// }

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			clawopen.toggle();
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			clawrotate.toggle();
		}
        pros::delay(10);
        }

}		
