#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <string>

ASSET(path_txt);
ASSET(next_txt);
ASSET(after_txt);
const double Pi = 3.14159265358979323846264;

const double toRad = Pi/180;

  pros::Motor left_front_motor(10, pros::E_MOTOR_GEAR_BLUE, true);
  pros::Motor left_back_motor(2, pros::E_MOTOR_GEAR_BLUE, true);
  pros::Motor left_center_motor(15, pros::E_MOTOR_GEAR_BLUE);
  pros::Motor right_front_motor(12, pros::E_MOTOR_GEAR_BLUE,false);
  pros::Motor right_middle_motor(16, pros::E_MOTOR_GEAR_BLUE,true);
  pros::Motor right_back_motor(1,  pros::E_MOTOR_GEAR_BLUE,false);

  pros::MotorGroup leftMotors({left_front_motor, left_center_motor, left_back_motor});
  pros::MotorGroup rightMotors({right_front_motor, right_middle_motor, right_back_motor});

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
lemlib::ControllerSettings linearController(20,//10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            120, // derivative gain (kD)
                                            3, // anti windup
                                            0.5, // small error range, in inches
                                            100, //FIX// small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, //FIX// large error range timeout, in milliseconds
                                            40 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             35, // derivative gain (kD)
                                             3, // anti windup
                                             1.5, // small error range, in degrees
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
        pros::lcd::print(3, "l front: %f", left_front_motor.get_position()); // print the heading
        pros::lcd::print(4, "r front: %f", right_front_motor.get_position()); // print the heading
        pros::lcd::print(5, "l back: %f", left_back_motor.get_position()); // print the heading
        pros::lcd::print(6, "r back: %f", right_back_motor.get_position()); // print the heading
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

  bool skills = true;
  auto intake = pros::MotorGroup({18});
  auto block =  pros::MotorGroup({19, -18});
  pros::ADIDigitalOut backwing('C');
  backwing.set_value(false);

  bool near = true;

  bool touch = true;


  if (!skills)


  {

    if (near) {

      chassis.tank(127, 127, 0);

      /*

      

      

      chassis.setPose(46.5, 55.5, 135);

      chassis.moveToPoint(57.5, 45.5, 2000, {}, false);

      backwing.set_value(true);

      pros::delay(500);

      chassis.moveToPoint(48, 52, 2000, {.forwards=false}, false);

      chassis.turnToHeading(90, 2000, {}, false);
      pros::delay(500);

      backwing.set_value(false);

      chassis.moveToPose(10, 60, 110, 5000, {.forwards=false}, false);

      */


      


      
      /*

      chassis.setPose(-38, 62, 180);

            wing2.set_value(true);

            pros::delay(700);

            //chassis.turnToHeading(270, 1000, {}, false);

           // chassis.moveToPose(-29, 58, 270, 1500, {.forwards=false}, false);

        //    pros::delay(700);

      //chassis.moveToPoint(-56, 30, 5000, {}, false);
      //chassis.tank(120, 120);
      //pros::delay(1500);

      

      //intake.move(-12000);

      
      //chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+10, 5000, {.forwards=false}, false);

      chassis.turnToPoint(-52, 48, 2000, {.forwards=true}, false);


      
      chassis.moveToPoint(-52, 48, 2000, {.forwards=true}, false);

      chassis.turnToPoint(180, 1500, {}, false);
      
      wing2.set_value(false);




      chassis.moveToPose(-62, 24.5, 180, 1500, {.forwards=true}, false);

      pros::delay(200);
      wing2.set_value(false);
      pros::delay(1300);

      chassis.moveToPose(-38, 62, 180, 2000, {.forwards=false}, false);
      chassis.moveToPose(-62, 24.5, 180, 2000, {.forwards=true}, true);
      pros::delay(100);

      intake.move(-12000);

      pros::delay(1900);

      //chassis.moveToPoint(-62, 37, 2000, {.forwards=false}, false);



      
      //block.move_voltage(12000);

      


      //chassis.turnToPoint(-31, 63, 5000, {.forwards=false}, false);


      chassis.turnToPoint(-8, 58, 2000, {.forwards=false}, false);
      */

      

    if (touch) {
      //chassis.moveToPoint(-8, 58, 2000, {.forwards=false}, false);

          //chassis.turnToPoint(-2, 61, 5000, {}, false);

    //chassis.moveToPoint(-1, 60, 5000, {.maxSpeed=40}, false);

    } else {
      chassis.tank(0, 0);
    }



    

    } else {

      chassis.setPose(-10.5, 60, 270);
      intake.move_voltage(-12000);
      chassis.moveToPoint(-7, 60, 3000, {.forwards=false}, false);
      pros::delay(1000);
      chassis.moveToPoint(-39, 60, 3000, {.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

      chassis.turnToPoint(-55, 42, 3000, {.forwards=true,.maxSpeed=70, .minSpeed=70}, true);
      chassis.moveToPoint(-55, 42, 3000, {.forwards=true,.maxSpeed=70, .minSpeed=70}, true);

      chassis.moveToPose(-59, 21, 180, 1500, {.forwards=true, .lead=0.2, .minSpeed=127, }, true);

      chassis.turnToPoint(-44, 49, 3000, {.forwards=false,.maxSpeed=70, .minSpeed=70}, true);
      chassis.moveToPoint(-44, 49, 3000, {.forwards=false,.maxSpeed=70, .minSpeed=70}, true);

      chassis.turnToPoint(-84, 26, 3000, {.forwards=false,.maxSpeed=70, .minSpeed=70}, false);
      backwing.set_value(true);

      intake.move_voltage(12000);

      chassis.moveToPoint(-52, 42 , 3000, {.forwards=false,.maxSpeed=70, .minSpeed=70}, false);
      chassis.turnToPoint(-67, 8,3000, {.forwards=false,.maxSpeed=127, .minSpeed=127}, false );

      backwing.set_value(false);

      chassis.moveToPose(-60, 40, 180, 2000, {.forwards=true,.maxSpeed=127, .minSpeed=127}, false );


      //chassis.turnToPoint(-59, 15, 3000, {.forwards=false,.maxSpeed=70, .minSpeed=70}, false);

      
            chassis.moveToPose(-62, 21, 180, 1500, {.forwards=false, .lead=0.3, .minSpeed=127, }, false);



      


    if (touch) {
      chassis.moveToPose(14, 58, 90, 5000, {.forwards=false}, false);

          //chassis.turnToPoint(-2, 61, 5000, {}, false);

    //chassis.moveToPoint(-1, 60, 5000, {.maxSpeed=40}, false);

    } else {
      chassis.tank(0, 0);
    }
    }



    /*

    auto catapult = pros::MotorGroup({13, -20});
    
    catapult.move_voltage(12000);
    pros::delay(50 * 1000);
    catapult.move_voltage(0);

    chassis.tank(100, 120, 70);
    chassis.cancelMotion();
    pros::delay(350);
    //leftMotors.move_voltage(12000);
  pros::MotorGroup e = pros::MotorGroup({10, 2});
  pros::MotorGroup g = pros::MotorGroup({12, 1});

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


      chassis.setPose(46.958, 56.013, 240);
      //chassis.moveToPose(37, 53, 240, 5000, {}, false);
  
      //chassis.moveToPose(30.204, 57.571, 270, 5000, {}, false)


      chassis.follow(path_txt, 17, 10000, true, false);

      chassis.setPose(-59.849, 23.809, 180);
      chassis.follow(next_txt, 10, 4000, false, false);
      chassis.setPose(-5.537, 30.169, 270);
      chassis.follow(after_txt, 10, 4000, true, false);

      chassis.tank(-67, -60);
      pros::delay(1500);
      wings.set_value(true);
      wing2.set_value(true);
      chassis.tank(127, 127);
      pros::delay(1500);
      wings.set_value(false);
      wing2.set_value(false);
      chassis.tank(-77,- 70);
      pros::delay(1500);
      wings.set_value(true);
      wing2.set_value(true);
      chassis.tank(127,127);
      pros::delay(1500);
      chassis.cancelAllMotions();
*/



      
  } else {
    auto catapult = pros::MotorGroup({13, -20});

    chassis.setPose(38, 60, 0);


    chassis.turnToPoint(61, 32, 3000, {.forwards=false}, false);
    //chassis.moveToPoint(53, 41, 3000, {.forwards=false}, false);
    //chassis.turnToPoint(57, 30, 3000, {.forwards=false}, false);
    chassis.moveToPose(59, 23, 0, 3000, {.forwards=false, .chasePower=10, .lead=0.4,  .minSpeed=100}, false);
    chassis.moveToPoint(61, 42, 3000, {}, false);
    chassis.turnToPoint(0, 17, 3000, {.forwards=false}, false); //30 33

    wings.set_value(true);

    catapult.move_voltage(-12000);


    pros::delay(24000);

    catapult.move_voltage(0);

    wings.set_value(false);

    chassis.turnToPoint(13, 32, 3000, {.forwards=false}, false);

    chassis.moveToPoint(19, 33, 3000, {.forwards=false, .minSpeed=100}, false);

    chassis.turnToHeading(180, 3000, {.forwards=true, .minSpeed=100}, false);

    wing2.set_value(true);

    wings.set_value(true);

    chassis.moveToPoint(19, -27 , 3000, {.forwards=true, .maxSpeed=127, .minSpeed=110}, false);

    chassis.moveToPoint(19, -39 , 3000, {.forwards=true, .minSpeed=120}, false);


    wings.set_value(false);
    wing2.set_value(false);

    chassis.moveToPoint(19, -30 , 3000, {.forwards=false, .maxSpeed=127, .minSpeed=100}, false);

    chassis.turnToPoint(45, -41, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=110}, false);

    chassis.moveToPoint(45, -41, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=110}, false);

    wings.set_value(true);

    chassis.turnToPoint(33, -54, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=110}, false);

    chassis.moveToPoint(33, -56, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=110}, false );

    wings.set_value(false);

    chassis.moveToPose(0, -60, 270, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=110}, false);

    chassis.moveToPoint(-35, -60, 3000, {.forwards=true,.maxSpeed=110, .minSpeed=110}, false);

    chassis.turnToPoint(-56, -41, 3000, {.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

            chassis.moveToPoint(-56, -47, 3000, {.forwards=true,  .maxSpeed=70, .minSpeed=70}, false);

    chassis.moveToPoint(-63, -26,  1000, {.forwards=true,  .maxSpeed=110, .minSpeed=110}, false);

    pros::delay(500);
    chassis.moveToPoint(-63, -40,  1000, {.forwards=false,  .minSpeed=50}, false);




    chassis.moveToPose(-63, -26, 0,  1000, {.forwards=true, .lead=0.3, .maxSpeed=110,  .minSpeed=127}, false);

        pros::delay(500);

    chassis.moveToPoint(-63, -40,  2000, {.forwards=false,  .minSpeed=50}, false);

        chassis.turnToPoint(-13, -32, 3000, {.forwards=true}, false);
    
    //wings.set_value(true);

    chassis.moveToPoint(-19, -20, 3000, {.forwards=true, .maxSpeed=110, .minSpeed=70,}, false);

    //wings.set_value(false);

    chassis.turnToPoint(-52, 48, 3000, {.forwards=true}, false);


    wings.set_value(true);

    
    chassis.moveToPose(-50, -1, 270, 3000, {.forwards=true, .lead = 0.3, .minSpeed=100}, false);

    chassis.moveToPoint(-37, -1, 3000, {.forwards=false, .minSpeed=100}, false);
    chassis.moveToPoint(-50, -1, 3000, {.forwards=true, .minSpeed=127}, false);
    pros::delay(500);
    chassis.moveToPoint(-15, -1, 3000, {.forwards=false, .minSpeed=100}, false);
    wings.set_value(false);
    chassis.turnToHeading(0, 3000, {.forwards=true, .minSpeed=100}, false);

        wings.set_value(true);
    chassis.moveToPose(-50, 12, 270, 10000, {.forwards=true, .lead = 0.7, .minSpeed=100}, false);

    chassis.moveToPoint(-37, 12, 3000, {.forwards=false, .minSpeed=100}, false);
    chassis.moveToPoint(-50, 12, 3000, {.forwards=true, .minSpeed=127}, false);
    pros::delay(500);
    chassis.moveToPoint(-15, 12, 3000, {.forwards=false, .minSpeed=100}, false);

    chassis.turnToPoint(-42, 44, 3000, {.forwards=true, .minSpeed=127}, false);

    chassis.moveToPoint(-39, 41, 3000, {.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

        wing2.set_value(true);

    chassis.moveToPoint(-47, 52, 3000, {.forwards=true, .maxSpeed=70, .minSpeed=70}, false);




    chassis.moveToPose(-49, 54, 270, 3000,{.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

    wing2.set_value(false);

    chassis.moveToPoint(-56, 53, 3000,{.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

    chassis.turnToPoint(-60, 49, 3000,{.forwards=true, .maxSpeed=70, .minSpeed=70}, false);

    chassis.moveToPose(-67, 31, 180, 2000,{.forwards=true, .maxSpeed=100, .minSpeed=100}, false);

    chassis.moveToPoint(-66, 45, 3000,{.forwards=false, .maxSpeed=50, .minSpeed=50}, false);

    chassis.moveToPoint(-67, 31, 3000,{.forwards=true, .maxSpeed=127, .minSpeed=127}, false);

    









    

    //chassis.turnToPoint(21, 12, 3000, {.forwards=true, .minSpeed=100}, false);
    

    

        

    //chassis.moveToPose(61, 32, 0, 2000, {.forwards=false});




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

  bool skills=false;


 

  // initializes the controller
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  // initialize motor groups in the chassis (drivetrain);
  // abstraction for the motors as a skid steer (tank) drivetrain
    //autonomous();
if (skills) {
        chassis.setPose(46.8, 57.5, 315);
      chassis.moveToPoint(chassis.getPose().x+7, chassis.getPose().y-7, 2000, {.forwards=false, .maxSpeed=100}, false);
      chassis.turnToHeadingLeft(248, 5000, {.forwards=true, .maxSpeed=70}, false);
          auto catapult = pros::MotorGroup({13, -20});
}

  // initializing motor groups
  auto intake = pros::MotorGroup({18});
  auto descore = pros::MotorGroup({3});
  auto catapult = pros::MotorGroup({-13, 20});




  // initialize pneumatics
  pros::ADIDigitalOut wings('A');

  pros::ADIDigitalOut wing2('B');

  pros::ADIDigitalOut backwing('C');

  pros::ADIDigitalOut pto('E');

  pros::ADIDigitalOut block('D');

  // set wings to close
  wings.set_value(false);
  wing2.set_value(false);
  backwing.set_value(false);
  pto.set_value(false);
  block.set_value(false);

  // prevents jerks
  intake.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  catapult.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);




  bool blockState = false;

  while (true) {
    // gets input from the joysticks
    chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
   // chassis.tank(master.getAnalog(okapi::ControllerAnalog::rightY) * 127, master.getAnalog(okapi::ControllerAnalog::leftY) * 127);

    // controls intakes
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == true) {
      intake.move_voltage(12000);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(-12000);
    } else {
      intake.move_voltage(0);
    }

    // controls catapult
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == true) {
      catapult.move_voltage(12000);
    } else {
      catapult.move_voltage(0);
    }
    if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == true))  {
      backwing.set_value(true);
    } 
    else if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == true)) {
      backwing.set_value(false);
    }

    // pnuematics logic
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == true) {
      wings.set_value(true);
      wing2.set_value(true);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) == true) {
      wings.set_value(false);
      wing2.set_value(false);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) == true) {
      block.set_value(true);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == true) {
      block.set_value(false);
    }


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == true) {
      pto.set_value(true);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == true) {
      pto.set_value(false);
    }





    // Delay in order to avoid a super-fast control loop
    pros::delay(20);
    
  }
  /*

      pros::ADIDigitalOut wings('A');

  pros::ADIDigitalOut wing2('B');

  // set wings to close
  wings.set_value(false);
  wing2.set_value(false);

  bool skills = true;
  auto intake = pros::MotorGroup({14});
  auto block =  pros::MotorGroup({19, -18});

  bool near = true;

  bool touch = false;


  if (!skills)


  {

    if (near) {

      chassis.setPose(-43, 62, 180);

            chassis.turnToPoint(-56, 30, 5000, {}, false);

      //chassis.moveToPoint(-56, 30, 5000, {}, false);
      chassis.tank(120, 120);
      pros::delay(1500);


      
      chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y+10, 5000, {.forwards=false}, false);

      wing2.set_value(true);

      intake.move(-12000);
      block.move_voltage(12000);

      


      chassis.turnToPoint(-36, 61, 5000, {.forwards=false}, false);


      chassis.tank(-120, -120);

      pros::delay(700);

      

      wing2.set_value(false);

    if (touch) {

          chassis.turnToPoint(-2, 61, 5000, {}, false);

    chassis.moveToPoint(-1, 60, 5000, {.maxSpeed=40}, false);

    } else {
      chassis.tank(0, 0);
    }



    

    } else {
      chassis.tank(127, 127);
      pros::delay(5000);
      chassis.tank(-100, -100);
      pros::delay(2000);
      chassis.cancelAllMotions();
    }



    /*

    auto catapult = pros::MotorGroup({13, -20});
    
    catapult.move_voltage(12000);
    pros::delay(50 * 1000);
    catapult.move_voltage(0);

    chassis.tank(100, 120, 70);
    chassis.cancelMotion();
    pros::delay(350);
    //leftMotors.move_voltage(12000);
  pros::MotorGroup e = pros::MotorGroup({10, 2});
  pros::MotorGroup g = pros::MotorGroup({12, 1});

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




      
  } else {
        //auto block =  pros::MotorGroup({19, -18});
      //block.move_voltage(12000);
      chassis.setPose(45.945, 56.904, 315);
      chassis.moveToPoint(chassis.getPose().x+7, chassis.getPose().y-7, 2000, {.forwards=false, .maxSpeed=100}, false);
      chassis.turnToHeadingLeft(248, 5000, {.forwards=true, .maxSpeed=70}, false);
          auto catapult = pros::MotorGroup({13, -20});

    
    catapult.move_voltage(12000);
    //pros::delay(34 * 1000);
    
      chassis.moveToPose(23, 59, 270, 2000, {}, true);
      pros::delay(2000);

      catapult.move_voltage(0);

      chassis.moveToPoint(-32, 59, 2000, {}, false);
      chassis.moveToPose(-57, 30, 180, 2000,{}, false);

      //-59, 53

      chassis.moveToPose(-59, 50 , 180, 1500, {.forwards=false}, false);

      chassis.moveToPoint(-59, 25, 1000, {}, false);

      chassis.moveToPose(-59, 50 , 180, 1500, {.forwards=false}, false);

      

      //chassis.moveToPoint(-59, 25, 1000, {}, false);
      chassis.tank(127, 127);
      pros::delay(700);

    


      //chassis.moveToPose(-36, 59, 270, 5000, {.forwards=false}, false);
      chassis.turnToHeading(180, 2000, {}, false);
      
      chassis.moveToPoint(-57, 36 , 2000, {.forwards=false}, false)

  
      //chassis.moveToPoint(-60, 49 , 2000, {.forwards=false}, false);
      chassis.turnToPoint(-16, 27, 2000, {}, false);
     chassis.moveToPoint(-16, 27 , 2000, {.forwards=true}, false);


      //chassis.turnToHeading(230, 5000, {}, false);

        wings.set_value(true);
      wing2.set_value(true);
           chassis.moveToPose(-43, 3 ,270,  2000, {.forwards=true}, false);

      //chassis.moveToPoint(-42, 9 , 5000, {.forwards=true}, false);
      //chassis.tank(127, 127);
      //pros::delay(2000);

      wings.set_value(false);
      wing2.set_value(false);

      pros::delay(500);

      chassis.moveToPoint(chassis.getPose().x+25, chassis.getPose().y, 2000, {.forwards=false}, false);

     wings.set_value(true);
      wing2.set_value(true);

      chassis.turnToHeading(270, 1000, {}, false);

      chassis.tank(127, 127);
      pros::delay(1000);

      //chassis.moveToPoint(-42, 9 , 5000, {.forwards=true}, false);

            wings.set_value(false);
      wing2.set_value(false);



      chassis.turnToHeading(280, 2000, {}, false);

      chassis.moveToPoint(chassis.getPose().x+25, chassis.getPose().y-2, 2000, {.forwards=false}, false);

           wings.set_value(true);
      wing2.set_value(true);

      chassis.turnToHeading(270, 2000, {}, false);
      

      chassis.tank(127, 127);
      pros::delay(1000);


                  wings.set_value(false);
      wing2.set_value(false);

      chassis.turnToHeading(270, 1000, {}, false);

      chassis.moveToPoint(chassis.getPose().x+25, chassis.getPose().y, 2000, {.forwards=false}, false);


      chassis.turnToPoint(-41, -40, 2000, {}, false);





      //wings.set_value(true);
      //wing2.set_value(true);
      chassis.moveToPoint(-41, -40, 2000, {}, false);

      chassis.turnToHeadingLeft(360, 5000, {.forwards=true, .maxSpeed=70}, false);

      chassis.moveToPoint(-55, -49 , 2000, {.forwards=false}, false);

            chassis.moveToPoint(-60, -30 , 2000, {.forwards=true}, false);

      //chassis.turnToHeading(190, 5000, {}, false);

      


      //chassis.moveToPoint(chassis.getPose().x+(-4*cos(210*toRad)), chassis.getPose().y+(-4*sin(210*toRad)), 50000, false, 70, false);


  }

  */



}
