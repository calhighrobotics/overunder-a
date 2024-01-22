#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>

ASSET(path_txt);

using namespace okapi::literals;


  pros::Motor left_front_motor(10, pros::E_MOTOR_GEAR_BLUE, true);
  pros::Motor left_back_motor(2, pros::E_MOTOR_GEAR_BLUE, true);
  pros::Motor right_front_motor(12, pros::E_MOTOR_GEAR_BLUE,false);
  pros::Motor right_back_motor(1,  pros::E_MOTOR_GEAR_BLUE,false);

  pros::MotorGroup leftMotors({left_front_motor, left_back_motor});
  pros::MotorGroup rightMotors({right_front_motor, right_back_motor});

  lemlib::Drivetrain drivetrain{
      &leftMotors,  // left drivetrain motors
      &rightMotors, // right drivetrain motors
      13.75,           // track width
      lemlib::Omniwheel::NEW_325,
      360,           // wheel rpm
      8
  };

  pros::Imu inertial_sensor(11); // port 2

  // odometry struct
  lemlib::OdomSensors sensors{
      nullptr, //&horizontal, // vertical tracking wheel 1
      nullptr ,// vertical tracking wheel 2
      nullptr, // horizontal tracking wheel 1
      nullptr, // we don't have a second tracking wheel, so we set it to nullptr
      &inertial_sensor // inertial sensor
  };

  // forward/backward PID
// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

  // create the chassis
  lemlib::Chassis chassis(drivetrain, linearController, angularController,
                          sensors);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::lcd::print(3, "bozo: %f", left_front_motor.get_position()); // print the heading
        pros::lcd::print(4, "bozo100: %f", right_front_motor.get_position()); // print the heading
        pros::delay(10);
    }
}
 
void initialize() { 
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0);
    pros::Task screenTask(screen); // create a task to print the position to the screen
  }
/*
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello Bozo");

  pros::lcd::register_btn1_cb(on_center_button);
}
*/

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::ADIDigitalOut wings('A');

  pros::ADIDigitalOut wing2('B');

  // set wings to close
  wings.set_value(false);
  wing2.set_value(false);

  bool skills = false;
  if (skills)


  {

    auto catapult = okapi::MotorGroup({13, -20});
    
    catapult.moveVoltage(12000);
    pros::delay(50 * 1000);
    catapult.moveVoltage(0);

    chassis.tank(100, 120, 70);
    chassis.cancelMotion();
    pros::delay(350);
    //leftMotors.move_voltage(12000);
  okapi::MotorGroup e = {10, 2};
  okapi::MotorGroup g = {12, 1};

  chassis.tank(127, 111);
  pros::delay(700);
  wings.set_value(true);
  wing2.set_value(true);
  chassis.tank(127, 111);
  pros::delay(4300);
  chassis.tank(-100, -100);
  pros::delay(2000);
  chassis.tank(127, 124);
  pros::delay(2000);
  chassis.tank(-100, -100);
  pros::delay(2000);
  chassis.tank(127, 124);
  pros::delay(2000);
  chassis.tank(-100, -100);
  pros::delay(2000);
  chassis.tank(127, 124);
  pros::delay(2000);
  chassis.tank(-100, -100);
  pros::delay(2000);
  chassis.tank(127, 124);


  chassis.cancelAllMotions();


    
    /*
      pros::ADIAnalogOut wing('A');
      pros::ADIAnalogOut wing2('B');

      wing.set_value(false);
      wing2.set_value(false);
      chassis.setPose(42.204, 57.571, 270);

      chassis.follow(path_txt, 15, 200000, true, true);
      wing.set_value(true);
      wing2.set_value(true);
      */
  } else {
      chassis.tank(127, 127);
      pros::delay(5000);
      chassis.tank(-100, -100);
      pros::delay(2000);
      chassis.cancelAllMotions();


  }







  

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // initializes the controller
  okapi::Controller master(okapi::ControllerId::master);
  // initialize motor groups in the chassis (drivetrain);
  auto drive = okapi::ChassisControllerBuilder()
                     .withMotors({10, 2}, {-12, -1})
                     .withDimensions({okapi::AbstractMotor::gearset::green},
                                     {{4_in, 12.5_in}, okapi::imev5GreenTPR})
                     .build();
  // abstraction for the motors as a skid steer (tank) drivetrain
  auto model =
      std::dynamic_pointer_cast<okapi::SkidSteerModel>(drive->getModel());


  // initializing motor groups
  auto intake = okapi::MotorGroup({14});
  auto descore = okapi::MotorGroup({3});
  auto catapult = okapi::MotorGroup({13, -20});
  auto block = okapi::MotorGroup({19, -18});

  // initializing controller buttons
  auto upArrow = okapi::ControllerButton(okapi::ControllerDigital::L1);
  auto downArrow = okapi::ControllerButton(okapi::ControllerDigital::L2);
  auto r1 = okapi::ControllerButton(okapi::ControllerDigital::R1);
  auto r2 = okapi::ControllerButton(okapi::ControllerDigital::R2);
  auto wingIn = okapi::ControllerButton(okapi::ControllerDigital::left);
  auto wingOut = okapi::ControllerButton(okapi::ControllerDigital::right);
  auto  dein = okapi::ControllerButton(okapi::ControllerDigital::A);
  auto dout = okapi::ControllerButton(okapi::ControllerDigital::Y);

  auto x = okapi::ControllerButton(okapi::ControllerDigital::X);
  auto b = okapi::ControllerButton(okapi::ControllerDigital::B);

  auto a = okapi::ControllerButton(okapi::ControllerDigital::A);
  auto y = okapi::ControllerButton(okapi::ControllerDigital::Y);

  // initialize pneumatics
  pros::ADIDigitalOut wings('A');

  pros::ADIDigitalOut wing2('B');

  // set wings to close
  wings.set_value(false);
  wing2.set_value(false);

  // prevents jerks
  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  bool blockState = false;

  while (true) {
    // gets input from the joysticks
    model->arcade(master.getAnalog(okapi::ControllerAnalog::rightX),
                  master.getAnalog(okapi::ControllerAnalog::leftY),
                  master.getAnalog(okapi::ControllerAnalog::leftX));

   // chassis.tank(master.getAnalog(okapi::ControllerAnalog::rightY) * 127, master.getAnalog(okapi::ControllerAnalog::leftY) * 127);

    // controls intakes
    if (upArrow.isPressed() == true) {
      intake.moveVoltage(-12000);
    } else if (downArrow.isPressed() == true) {
      intake.moveVoltage(12000);
    } else {
      intake.moveVoltage(0);
    }

    // controls catapult
    if (r1.isPressed() == true) {
      catapult.moveVoltage(12000);
    } else {
      catapult.moveVoltage(0);
    }


    if (x.isPressed() == true) {
      blockState = true;
    } else if (b.isPressed() == true) {
      blockState = false;
    }

    if (blockState) block.moveVoltage(12000);
    else block.moveVoltage(0);


      if (dein.isPressed() == true) {
      descore.moveVoltage(12000);
    } else if (dout.isPressed() == true) {
      descore.moveVoltage(-12000);
    } else {
      descore.moveVoltage(0);
    }

    // pnuematics logic
    if (wingOut.isPressed() == true) {
      wings.set_value(true);
      wing2.set_value(true);
    } else if (wingIn.isPressed() == true) {
      wings.set_value(false);
      wing2.set_value(false);
    }





    // Delay in order to avoid a super-fast control loop
    pros::delay(20);
  }
}
