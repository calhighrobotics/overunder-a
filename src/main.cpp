#include "main.h"
#include "const.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <string>

using namespace okapi::literals;

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
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello Bozo");

  pros::lcd::register_btn1_cb(on_center_button);
}

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

  if (skills) {
    // initialize motor groups in the chassis (drivetrain)
    okapi::MotorGroup mGroup = {-2, 4};
    auto chassis = okapi::ChassisControllerBuilder()
                       .withMotors({topLMot, botLMot}, {topRMot, botRMot})
                       .withDimensions({okapi::AbstractMotor::gearset::green},
                                       {{4_in, 12.5_in}, okapi::imev5GreenTPR})
                       .build();
    // abstraction for the motors as a skid steer (tank) drivetrain
    auto model =
        std::dynamic_pointer_cast<okapi::SkidSteerModel>(chassis->getModel());

    // initializing motor groups
    auto intake = okapi::MotorGroup({11,-12});
    auto catapult = okapi::MotorGroup({cataMot});

    // initialize pneumatics
    pros::ADIDigitalOut wings(wingsADIPort);

    // set wings to close
    wings.set_value(false);

    // prevents jerks
    intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    intake.moveVoltage(-12000);
    pros::delay(200);
    intake.moveVoltage(0);

    mGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    mGroup.moveVoltage(12000);
    pros::delay(710);
    mGroup.moveVoltage(0);
	catapult.moveVoltage(12000);
    
  } else {
	    // initialize motor groups in the chassis (drivetrain)
    okapi::MotorGroup mGroup = {topLMot, botLMot, -2, 4};
    auto chassis = okapi::ChassisControllerBuilder()
                       .withMotors({topLMot, botLMot}, {topRMot, botRMot})
                       .withDimensions({okapi::AbstractMotor::gearset::green},
                                       {{4_in, 12.5_in}, okapi::imev5GreenTPR})
                       .build();
    // abstraction for the motors as a skid steer (tank) drivetrain
    auto model =
        std::dynamic_pointer_cast<okapi::SkidSteerModel>(chassis->getModel());

    // initializing motor groups
    auto intake = okapi::MotorGroup({intakeMot});
    auto catapult = okapi::MotorGroup({cataMot});

    // initialize pneumatics
    pros::ADIDigitalOut wings(wingsADIPort);

    // set wings to close
    wings.set_value(false);

    // prevents jerks
    intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    catapult.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    //intake.moveVoltage(-12000);
    //pros::delay(200);
   // intake.moveVoltage(0);

    mGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    mGroup.moveVoltage(12000);
    pros::delay(2000);
    mGroup.moveVoltage(0);
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
  // initialize motor groups in the chassis (drivetrain)
  auto chassis = okapi::ChassisControllerBuilder()
                     .withMotors({topLMot, botLMot}, {topRMot, botRMot})
                     .withDimensions({okapi::AbstractMotor::gearset::green},
                                     {{4_in, 12.5_in}, okapi::imev5GreenTPR})
                     .build();
  // abstraction for the motors as a skid steer (tank) drivetrain
  auto model =
      std::dynamic_pointer_cast<okapi::SkidSteerModel>(chassis->getModel());

  // initializing motor groups
  auto intake = okapi::MotorGroup({11,-12});
  auto catapult = okapi::MotorGroup({cataMot});

  // initializing controller buttons
  auto upArrow = okapi::ControllerButton(okapi::ControllerDigital::L1);
  auto downArrow = okapi::ControllerButton(okapi::ControllerDigital::L2);
  auto r1 = okapi::ControllerButton(okapi::ControllerDigital::R1);
  auto r2 = okapi::ControllerButton(okapi::ControllerDigital::R2);
  auto wingIn = okapi::ControllerButton(okapi::ControllerDigital::left);
  auto wingOut = okapi::ControllerButton(okapi::ControllerDigital::right);

  // initialize pneumatics
  pros::ADIDigitalOut wings(wingsADIPort);

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
      intake.moveVoltage(12000);
    } else if (downArrow.isPressed() == true) {
      intake.moveVoltage(-12000);
    } else {
      intake.moveVoltage(0);
    }

    // controls catapult
    if (r1.isPressed() == true) {
      catapult.moveVoltage(12000);
    } else {
      catapult.moveVoltage(0);
    }

    // pnuematics logic
    if (wingOut.isPressed() == true) {
      wings.set_value(true);
    } else if (wingIn.isPressed() == true) {
      wings.set_value(false);
    }

    // Delay in order to avoid a super-fast control loop
    pros::delay(20);
  }
}
