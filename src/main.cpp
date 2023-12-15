#include "main.h"
#include "const.h"
#include "lemlib/chassis/chassis.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <string>

ASSET(path_txt);

using namespace okapi::literals;


  pros::Motor left_front_motor(10, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor left_back_motor(2, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor right_front_motor(12, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor right_back_motor(1, pros::E_MOTOR_GEARSET_06, false);

  pros::MotorGroup leftMotors({left_front_motor, left_back_motor});
  pros::MotorGroup rightMotors({right_front_motor, right_back_motor});

  pros::Rotation horizontalEnc(2);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot
  lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -7);

  pros::Rotation horizontal2Enc(12);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot
  lemlib::TrackingWheel horizontal2(&horizontal2Enc, lemlib::Omniwheel::NEW_275, 7);

  lemlib::Drivetrain drivetrain{
      &leftMotors,  // left drivetrain motors
      &rightMotors, // right drivetrain motors
      14,           // track width
      lemlib::Omniwheel::NEW_325,
      360,           // wheel rpm
      2
  };

  pros::Imu inertial_sensor(11); // port 2

  // odometry struct
  lemlib::OdomSensors sensors{
      nullptr, // vertical tracking wheel 1
      nullptr, // vertical tracking wheel 2
      &horizontal, // horizontal tracking wheel 1
      nullptr, // we don't have a second tracking wheel, so we set it to nullptr
      &inertial_sensor // inertial sensor
  };

  // forward/backward PID
  lemlib::ControllerSettings lateralController{
      10,   // kP
      50,  // kD
      1,   // smallErrorRange
      100, // smallErrorTimeout
      3,   // largeErrorRange
      500, // largeErrorTimeout
      5    // slew rate
  };

  // turning PID
  lemlib::ControllerSettings angularController{
      5,   // kP //WAS 4
      50,  // kD
      1,   // smallErrorRange
      100, // smallErrorTimeout
      3,   // largeErrorRange
      500, // largeErrorTimeout
      40   // slew rate
  };

  // create the chassis
  lemlib::Chassis chassis(drivetrain, lateralController, angularController,
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

  bool skills = false;
  if (skills == true)
  {
      pros::ADIAnalogOut wing('A');

      wing.set_value(false);

      auto catapult = okapi::MotorGroup({-13, 20});

      catapult.moveVoltage(12000);

      pros::delay(50 * 1000);

      catapult.moveVoltage(0);
      chassis.setPose(0, 0 , 326);

      chassis.turnTo(0, 1, 1000, true, 80, false);

      chassis.setPose(46, 59, 270);

      chassis.follow(path_txt, 15, 20000, true, true);
      wing.set_value(true);
      pros::delay(4*1000);
      wing.set_value(true);

      pros::delay(2000);
  }

  okapi::MotorGroup e = {10, 2};
  okapi::MotorGroup g = {12, 1};

  e.moveVoltage(12000);
  g.moveVoltage(12000);
  pros::delay(2000);

  e.moveVoltage(0);
  g.moveVoltage(0);





  

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
  auto chassis = okapi::ChassisControllerBuilder()
                     .withMotors({10, 2}, {-12, -1})
                     .withDimensions({okapi::AbstractMotor::gearset::green},
                                     {{4_in, 12.5_in}, okapi::imev5GreenTPR})
                     .build();
  // abstraction for the motors as a skid steer (tank) drivetrain
  auto model =
      std::dynamic_pointer_cast<okapi::SkidSteerModel>(chassis->getModel());


  // initializing motor groups
  auto intake = okapi::MotorGroup({3});
  auto catapult = okapi::MotorGroup({-13, 20});
  auto block = okapi::MotorGroup({19, -18});

  // initializing controller buttons
  auto upArrow = okapi::ControllerButton(okapi::ControllerDigital::L1);
  auto downArrow = okapi::ControllerButton(okapi::ControllerDigital::L2);
  auto r1 = okapi::ControllerButton(okapi::ControllerDigital::R1);
  auto r2 = okapi::ControllerButton(okapi::ControllerDigital::R2);
  auto wingIn = okapi::ControllerButton(okapi::ControllerDigital::left);
  auto wingOut = okapi::ControllerButton(okapi::ControllerDigital::right);

  auto x = okapi::ControllerButton(okapi::ControllerDigital::X);
  auto b = okapi::ControllerButton(okapi::ControllerDigital::B);

  auto a = okapi::ControllerButton(okapi::ControllerDigital::A);
  auto y = okapi::ControllerButton(okapi::ControllerDigital::Y);

  // initialize pneumatics
  pros::ADIDigitalOut wings('A');

  pros::ADIDigitalOut end('B');

  end.set_value(false);

  // set wings to close
  wings.set_value(false);

  // prevents jerks
  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  while (true) {
    // gets input from the joysticks
    model->arcade(master.getAnalog(okapi::ControllerAnalog::rightX),
                  master.getAnalog(okapi::ControllerAnalog::leftY),
                  master.getAnalog(okapi::ControllerAnalog::leftX));

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
      block.moveVoltage(6000);
    } else if (b.isPressed() == true) {
      block.moveVoltage(-6000);
    } else {
      block.moveVoltage(0);
    }

    // pnuematics logic
    if (wingOut.isPressed() == true) {
      wings.set_value(true);
    } else if (wingIn.isPressed() == true) {
      wings.set_value(false);
    }

    if (a.isPressed() == true) {
      end.set_value(true);
    } else if (y.isPressed() == true) {
      end.set_value(false);
    }



    // Delay in order to avoid a super-fast control loop
    pros::delay(20);
  }
}
