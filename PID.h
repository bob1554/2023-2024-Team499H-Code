#include "vex.h"
#include <iostream>

namespace PID {

void turnPID(double degrees){
   
  double turnkP = 3;
  double turnkI = 0.00;
  double turnkD = 0.0;
  

  int turnError = 0;
  int turnPrevError = 0;
  int turnDerivative;
  int turnTotalError = 0;

  int stopTollerance = 2;

  Inertial_Sensor.calibrate();


  do{
    int currentPosition = Inertial_Sensor.rotation();

    std::cout << turnError << "\n";
    
    turnError = degrees - currentPosition;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double driveTrainTurnPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    turnPrevError = turnError;  

    leftFront.spin(forward, driveTrainTurnPower/ 50, rpm);
    leftBack.spin(forward, driveTrainTurnPower / 50, rpm);
    leftMid.spin(forward, driveTrainTurnPower / 50, rpm);
    rightFront.spin(reverse, driveTrainTurnPower /  50, rpm);
    rightBack.spin(reverse, driveTrainTurnPower / 50, rpm);
    rightMid.spin(reverse, driveTrainTurnPower / 50, rpm);

    wait(5, timeUnits::msec);
  }
  while(abs(turnError) > stopTollerance);

  leftFront.stop(brake);
  leftBack.stop(brake);
  leftMid.stop(brake);
  rightFront.stop(brake);
  rightBack.stop(brake);
  rightMid.stop(brake);

}

void linearPID(double rot, double reduction, bool drift){
  double linearkP = 12.5;
  double linearkI = 0.00007;
  double linearkD = 0.05;

  int linearError = 0;
  int linearPrevError = 0;
  int linearDerivative;
  int linearTotalError = 0;


  int stopTollerance = 10;
  double kDrift = 200;

  rightFront.resetRotation();
  leftFront.resetRotation();
 


  double intOrientation = Inertial_Sensor.rotation();

  do{
    int leftPosition = leftFront.position(rotationUnits::deg);
    int rightPosition = rightFront.position(rotationUnits::deg);

    int avgPosition = (leftPosition + rightPosition) / 2;

    std::cout << linearError << "\n";

    linearError = rot - avgPosition;
    linearDerivative = linearError - linearPrevError;
    linearTotalError += linearError;

    double driveTrainMotorPower = linearError * linearkP + linearDerivative * linearkD + linearTotalError * linearkI;

    linearPrevError = linearError; 

    double driftCounter;

    if(drift){
      if(rot < 0){
        driftCounter = (Inertial_Sensor.rotation() + intOrientation) * kDrift;
      }else {
        driftCounter = -1 * (Inertial_Sensor.rotation() - intOrientation) * kDrift; 
      }
    } else {
      driftCounter = 0;
    }
    leftFront.spin(forward, (driveTrainMotorPower + driftCounter) / (100 * reduction), rpm);
    leftBack.spin(forward,(driveTrainMotorPower + driftCounter) / (100 * reduction), rpm);
    leftMid.spin(forward, (driveTrainMotorPower + driftCounter) / (100 * reduction), rpm);
    rightFront.spin(forward, (driveTrainMotorPower - driftCounter) / (100 * reduction), rpm); 
    rightBack.spin(forward, (driveTrainMotorPower - driftCounter) / (100 * reduction), rpm); 
    rightMid.spin(forward, (driveTrainMotorPower - driftCounter) / (100 * reduction), rpm); 

    wait(5, timeUnits::msec);
  }
  while(linearError > stopTollerance || linearError < -1 * stopTollerance);
    leftFront.stop(brake);
    leftBack.stop(brake);
    leftMid.stop(brake);
    rightFront.stop(brake);
    rightBack.stop(brake);
    rightMid.stop(brake);
}


}