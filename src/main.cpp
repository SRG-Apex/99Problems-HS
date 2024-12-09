/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Project:      241E High Stakes                                          */
/*    Author(s):    Andrew Bobay, Heer Patel                                  */
/*    Created:      Aug 2nd 2024                                              */
/*    Modified:     Dec. 9th 2024                                             */
/*    Description:  241e's code for the Vex High Stakes 2024-2025             */
/*                  season                                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"

using namespace vex;

// OLD PORTS PLEASE CHANGE!!!!!!!!!!!!!!!

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller(primary);

motor driveFR = motor(PORT18, ratio6_1, false);
motor driveBR = motor(PORT17, ratio6_1, false);
motor driveTR = motor(PORT16, ratio6_1, true);
motor driveFL = motor(PORT11, ratio6_1, true);
motor driveBL = motor(PORT13, ratio6_1, true);
motor driveTL = motor(PORT12, ratio6_1, false);

motor_group driveRight = motor_group(driveFR, driveBR, driveTR);
motor_group driveLeft  = motor_group(driveFL, driveBL, driveTL);

motor_group drive = motor_group(driveFR, driveBR, driveTR, driveFL, driveBL, driveTL);

inertial IMU = inertial(PORT14);
double gearRatio = 0.6;
double WheelC = 10.21;
smartdrive Drivetrain = smartdrive(driveLeft, driveRight, IMU, 260, 320, 40, mm, 0.6);

motor intakeU = motor(PORT20, ratio6_1, false); //11 W
motor intakeL = motor(PORT19, ratio18_1, true); //5.5 W
motor_group Intake = motor_group(intakeU, intakeL); //Never run based on volts/watts only percentage

pneumatics clamp1 = pneumatics(Brain.ThreeWirePort.A); //clampBool
pneumatics clamp2 = pneumatics(Brain.ThreeWirePort.B); //clampBool

pneumatics ringArm = pneumatics(Brain.ThreeWirePort.C);

motor WSArm = motor(PORT10, ratio18_1, false);



bool driveRightBool = true;
bool driveLeftBool = true;
bool IntakeBool = true;
bool clampBool = false;
bool armBool = false;
bool WSArmBool = true;



// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  // IMU CALIBRATION
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  IMU.calibrate(0);

  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (IMU.isCalibrating()) {
  }
  Brain.Screen.print("Inertial Calibration Complete");
  Intake.setVelocity(100, percent);
  driveLeft.resetPosition();
  driveRight.resetPosition();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void PIDdrive(double target, double kv){
  drive.setVelocity(100,percent);
    double kp = .505;
    double ki = .11;
    double kd = .01;
    double voltage = getVoltage(target, kv);
    double error;
    double integral;
    double derivative;
    double prevError;
    double output;
    double currPos = 0;
    int direction = fabs(target)/target; //return 1 or -1
    target = ((target / WheelC) * 360)*(gearRatio); // tune target to rotations
    

    // Keep driving until the average encoder value is equal to the target
    while (currPos != fabs(target) * direction) {
      currPos = (driveRight.position(deg) + driveLeft.position(deg))/2;
      error = (target - currPos) * direction;
      integral += error;
      derivative = (error - prevError);

      output = (kp * error + ki * integral + kd * derivative) * direction;
      Brain.Screen.setPenColor(255);
      Brain.Screen.print(output);
      drive.spinFor(output, deg);
      wait(20,msec);
      prevError = error;
  }}

double getVoltage(double target, double kv){

  double kp = 1;
  double currPos = 0;
  target = ((target / WheelC) * 360)*(gearRatio); // tune target to rotations
  double volts;

  while(currPos < target){
  currPos = (driveRight.position(deg) + driveLeft.position(deg))/2;


  volts = 12*((target - currPos)/target)*kp;

  drive.spin(fwd, volts, percent);

  wait(10, msec);

  }


}


void autonomous(void) {

  // ..........................................................................

  // Ring Side Red and Goal Side Blue
// Drivetrain.setDriveVelocity(20, percent);
//   clamp1.open();
//   clamp2.open();
// Drivetrain.driveFor(-32, inches);
// clamp1.close();
// clamp2.close();
// Intake.spin(fwd);
// wait(.5, sec);
//     Intake.spin(reverse);
//     wait(.5, sec);
//     Intake.stop();
//       Intake.spin(fwd);
//     Drivetrain.turnFor(90, deg);
//     Drivetrain.setDriveVelocity(30, percent); 
//     Drivetrain.driveFor(30, inches);
//       wait(2, sec);
//     Intake.spin(reverse);
//     wait(.25, sec);
//     Intake.stop();
//       Intake.spin(fwd);
//     Drivetrain.turnFor(-20, deg);
//     Intake.stop();
//     Drivetrain.driveFor(-40, inches);


  // ..........................................................................

  // Ring side Blue AND Goal Side Red
// Drivetrain.setDriveVelocity(20, percent);
//   clamp1.open();
//   clamp2.open();
// Drivetrain.driveFor(-32, inches);
// clamp1.close();
// clamp2.close();
// Intake.spin(fwd);
// wait(.5, sec);
//     Intake.spin(reverse);
//     wait(.5, sec);
//     Intake.stop();
//       Intake.spin(fwd);
//     Drivetrain.turnFor(-90, deg);
//     Drivetrain.setDriveVelocity(30, percent);
//     Drivetrain.driveFor(30, inches);
//       wait(2, sec);
//     Intake.spin(reverse);
//     wait(.25, sec);
//     Intake.stop();
//       Intake.spin(fwd);
//     Drivetrain.turnFor(20, deg);
//     Intake.stop();
//     Drivetrain.driveFor(-40, inches);


  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  Drivetrain.setDriveVelocity(100, percent);
  Intake.stop();
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................

      int drivetrainRightSideSpeed = (Controller1.Axis3.position() - 0.7*Controller1.Axis1.position());
      int drivetrainLeftSideSpeed = (Controller1.Axis3.position() + 0.7* Controller1.Axis1.position());


      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        if (driveLeftBool) {
          driveLeft.stop();
          driveLeftBool = false;
        }
      } else {
        driveLeftBool = true;
      }
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        if (driveRightBool) {
          driveRight.stop();
          driveRightBool = false;
        }
      } else {
        driveRightBool = true;
      }

      if (driveLeftBool) {
        driveLeft.setVelocity(drivetrainLeftSideSpeed, percent);
        driveLeft.spin(forward);
      }

      if (driveRightBool) {
        driveRight.setVelocity(drivetrainRightSideSpeed, percent);
        driveRight.spin(forward);
      }

      if (Controller1.ButtonR1.pressing()) {
        Intake.spin(reverse);
        IntakeBool = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Intake.spin(fwd);
        IntakeBool = false;
      } else if (!IntakeBool) {
        Intake.stop();
        IntakeBool = true;
      }


      if (Controller1.ButtonRight.pressing()) {
        WSArm.spin(reverse);
        WSArmBool = false;
      } else if (Controller1.ButtonDown.pressing()) {
        WSArm.spin(fwd);
        WSArmBool = false;
      } else if (!WSArmBool) {
        WSArm.stop();
        WSArmBool = true;
      }

        if (Controller1.ButtonL2.pressing()) {
            clampBool = !clampBool;
            while (Controller1.ButtonL2.pressing()){}
            if (clampBool) {
                clamp1.open();
                clamp2.open();
            } else {
                clamp1.close();
                clamp2.close();
                            }
        }

        if (Controller1.ButtonL1.pressing()) {
            armBool = !armBool;
            while (Controller1.ButtonL1.pressing()){}
            if (armBool) {
                ringArm.open();
            } else {
                ringArm.close();
            }
        }



    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
