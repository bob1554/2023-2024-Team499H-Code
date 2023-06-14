/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFront            motor         5               
// leftMid              motor         6               
// leftBack             motor         7               
// rightFront           motor         9               
// rightMid             motor         3               
// rightBack            motor         4               
// Inertial_Sensor      inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "../include/PID.h"

using namespace vex;
using namespace PID;

// A global instance of competition
competition Competition;

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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void autonomous(void) {
  Inertial_Sensor.calibrate();
  PID::turnPID(90);
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
double numCutoff(double num, double cutoff){
  if(num > cutoff)
    num = cutoff;
  else if(num < -cutoff)
    num = -cutoff;
  return num;
}

int rightPosition(){
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Right %f", rightFront.position(degrees));
  return 1;
}

int leftPosition(){
  Brain.Screen.setCursor(2,1);
  Brain.Screen.print("Left %f", leftFront.position(degrees));
  return 1;
}

int avgPosition(){
  Brain.Screen.setCursor(3,1);
  Brain.Screen.print("Avg %f", (leftFront.position(degrees) + rightFront.position(degrees)) / 2);
  return 1;
}

void usercontrol(void) {

  int jvaluesLFM;
  int jvaluesLMM;
  int jvaluesLBM;
  int jvaluesRFM;
  int jvaluesRBM;
  int jvaluesRMM;
  
  while (1) {

    rightPosition();
    leftPosition();
    avgPosition();
    
    jvaluesLFM = Controller1.Axis3.position() + (Controller1.Axis1.position() * 0.8);
    jvaluesLBM = Controller1.Axis3.position() + (Controller1.Axis1.position() * 0.8);
    jvaluesLMM = Controller1.Axis3.position() + (Controller1.Axis1.position() * 0.8);
    jvaluesRFM = Controller1.Axis3.position() - (Controller1.Axis1.position() * 0.8);
    jvaluesRMM = Controller1.Axis3.position() - (Controller1.Axis1.position() * 0.8);
    jvaluesRBM = Controller1.Axis3.position() - (Controller1.Axis1.position() * 0.8);

    leftFront.spin(fwd, numCutoff(jvaluesLFM, 100), pct);
    leftMid.spin(fwd, numCutoff(jvaluesLMM, 100), pct);
    leftBack.spin(fwd, numCutoff(jvaluesLBM, 100), pct);
    rightFront.spin(fwd, numCutoff(jvaluesRFM, 100), pct);
    rightMid.spin(fwd, numCutoff(jvaluesRMM, 100), pct);
    rightBack.spin(fwd, numCutoff(jvaluesRBM, 100), pct);

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
