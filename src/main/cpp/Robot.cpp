// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoMobility, kAutoMobility);
  m_chooser.AddOption(kAutoMobilityWire, kAutoMobilityWire);
  m_chooser.AddOption(kAutoChargeDist, kAutoChargeDist);
  m_chooser.AddOption(kAutoChargeGyro, kAutoChargeGyro); 
  m_chooser.AddOption(kAutoForward, kAutoForward); 
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  compressor.EnableDigital();

}

void Robot::RobotPeriodic() {

  frc::SmartDashboard::PutNumber("Lift Encoder", lift1Encoder.GetDistance());
  /*frc::SmartDashboard::PutNumber("Left Wheel Output", power.GetCurrent(5) );  
  frc::SmartDashboard::PutNumber("Right Wheel Output", power.GetCurrent(6) );
  frc::SmartDashboard::PutNumber("Cone Wheel Output", power.GetCurrent(7) );*/ 
  frc::SmartDashboard::PutBoolean("Limit Switch Value", LiftSwitch.Get());
  frc::SmartDashboard::PutNumber("Left Drive Encoder", leftdriveEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Right Drive Encoder", rightdriveEncoder.GetDistance());
  //frc::SmartDashboard::PutNumber("Encoder Lock Value", lockvalue);
  //frc::SmartDashboard::PutBoolean("Encoder Lock Bool", lockbool);
  frc::SmartDashboard::PutNumber("Auto Timer", (double)AutoTimer.Get());
  frc::SmartDashboard::PutNumber("Pitch", gyro.GetPitch());
  frc::SmartDashboard::PutNumber("Speed Controller L", lDrive.Get());
  frc::SmartDashboard::PutNumber("Speed Controller R", rDrive.Get());
  frc::SmartDashboard::PutNumber("Speed: ", Speed);
  frc::SmartDashboard::PutNumber("True Pitch", truepitch);
  if(LiftSwitch.Get()) {
  lift1Encoder.Reset();
  }
  truepitch = gyro.GetPitch() - startpitch;
}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if ( m_autoSelected == kAutoMobility) {autoMode = AutoMobility;}
  else if ( m_autoSelected == kAutoMobilityWire) {autoMode = AutoMobilityWire;}
  else if ( m_autoSelected == kAutoChargeDist) {autoMode = AutoChargeDist;}
  else if ( m_autoSelected == kAutoChargeGyro) {autoMode = AutoChargeGyro;}
  else if ( m_autoSelected == kAutoForward) {autoMode = AutoForward;}
  else if ( m_autoSelected == kAutoDoNothing) {autoMode = AutoDoNothing;}
  else {autoMode = AutoDoNothing;}
  leftdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
  lift1Encoder.SetDistancePerPulse(1./128.);
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
  startpitch = gyro.GetPitch();
  
}

void Robot::AutonomousPeriodic() { 
switch (autoMode) {


//Auto Mobility Normal
case AutoMobility:
switch (AutoStage) {


case 0:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
  AutoStage = 1;
  }
else {lift1motor.Set(-.7);}
if(lift1Encoder.GetDistance() >-68 && lift1Encoder.GetDistance() <-65){
lift2.Set(lift2.kReverse);
AutoTimer.Start();
}
break; 


case 1:
if((double)AutoTimer.Get() >= .75){
lift2.Set(lift2.kOff);
AutoStage = 2;
}
break;


case 2:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(.4, 6, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoTimer.Reset();
    AutoStage = 3;
}
break;


case 3:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}
if((double)AutoTimer.Get() >= .25){
  lift2.Set(lift2.kForward);
  clamp.Set(clamp.kReverse);
  if(lift1Encoder.GetDistance() <= -75){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 4;
  }
}
break;


case 4:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(-.4, 9, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 5;
}
break;

case 5:
if(LiftSwitch.Get() == true) {
    lift1motor.Set(0);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    Speed = .7;
    AutoStage = 6;
}
  else {lift1motor.Set(.4);}
break;


case 6:
  if (DistanceDrive(-.7, 93, true) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 7;
}
break;


case 7:
autoMode = AutoDoNothing;
break;

}
break;


//Auto Mobility Wire
case AutoMobilityWire:{
switch (AutoStage) {


case 0:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
  AutoStage = 1;
  }
else {lift1motor.Set(-.7);}
if(lift1Encoder.GetDistance() >-68 && lift1Encoder.GetDistance() <-65){
lift2.Set(lift2.kReverse);
AutoTimer.Start();
}
break; 


case 1:
if((double)AutoTimer.Get() >= .75){
lift2.Set(lift2.kOff);
AutoStage = 2;
}
break;


case 2:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(.4, 6, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoTimer.Reset();
    AutoStage = 3;
}
break;


case 3:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}
if((double)AutoTimer.Get() >= .25){
  lift2.Set(lift2.kForward);
  clamp.Set(clamp.kReverse);
  if(lift1Encoder.GetDistance() <= -75){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 4;
  }
}
break;


case 4:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(-.4, 9, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 5;
}
break;

case 5:
if(LiftSwitch.Get() == true) {
    lift1motor.Set(0);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 6;
}
  else {lift1motor.Set(.4);}
break;


case 6:
  if (DistanceDrive(-.7, 45, false) == DONE) {
    drive.TankDrive(0,0,false);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 7;
  }
break;


case 7:
  if (DistanceDrive(-.4, 23, false) == DONE) {
    drive.TankDrive(0,0,false);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 8;
  }
break;


case 8:
  if (DistanceDrive(-.7, 65, true) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 9;
  }
break;
case 9:
autoMode = AutoDoNothing;
break;

}
break;
}
break;


//AutoCharge Distance
case AutoChargeDist:
switch (AutoStage) {


case 0:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
  AutoStage = 1;
  }
else {lift1motor.Set(-.7);}
if(lift1Encoder.GetDistance() >-68 && lift1Encoder.GetDistance() <-65){
lift2.Set(lift2.kReverse);
AutoTimer.Start();
}
break; 


case 1:
if((double)AutoTimer.Get() >= .75){
lift2.Set(lift2.kOff);
AutoStage = 2;
}
break;


case 2:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(.4, 6, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoTimer.Reset();
    AutoStage = 3;
}
break;


case 3:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}
if((double)AutoTimer.Get() >= .25){
  lift2.Set(lift2.kForward);
  clamp.Set(clamp.kReverse);
  if(lift1Encoder.GetDistance() <= -75){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 4;
  }
}
break;


case 4:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(-.4, 9, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 5;
}
break;

case 5:
if(LiftSwitch.Get() == true) {
    lift1motor.Set(0);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 6;
}
  else {lift1motor.Set(.4);}
break;


case 6:
  if (DistanceDrive(-.7, 22, false) == DONE) {
    drive.TankDrive(0,0,false);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    Speed = .7;
    AutoStage = 7;
  }
break;


case 7:
if(leftdriveEncoder.GetDistance() >= -61 && rightdriveEncoder.GetDistance() <= 61){
SpeedDrive();
} 
else{
  drive.TankDrive(0,0,false);
  brake.Set(brake.kOff);
  AutoStage = 8;
}
break;


case 8:
brake.Set(brake.kForward);
AutoStage = 9;
break;

case 9:
autoMode = AutoDoNothing;

break;

}
break;


//AutoCharge Gyro
case AutoChargeGyro:{
  std::cout << "AutoStage: " << AutoStage << std::endl;
switch (AutoStage) {


case 0:
drive.TankDrive(0,0,false);
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
  AutoStage = 1;
  }
else {lift1motor.Set(-.7);}
if(lift1Encoder.GetDistance() >-68 && lift1Encoder.GetDistance() <-65){
lift2.Set(lift2.kReverse);
AutoTimer.Start();
}
break; 


case 1:
drive.TankDrive(0,0,false);
if((double)AutoTimer.Get() >= .75){
lift2.Set(lift2.kOff);
AutoStage = 2;
}
break;


case 2:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(.4, 6, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoTimer.Reset();
    AutoStage = 3;
}
break;


case 3:
drive.TankDrive(0,0,false);
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}
if((double)AutoTimer.Get() >= .25){
  lift2.Set(lift2.kForward);
  clamp.Set(clamp.kReverse);
  if(lift1Encoder.GetDistance() <= -75){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 4;
  }
}
break;


case 4:
if(lift1Encoder.GetDistance() <= -75) {
  lift1motor.Set(0);
  }
else {lift1motor.Set(-.7);}

if (DistanceDrive(-.4, 9, false) == DONE) {
    drive.TankDrive(0,0,false);
    AutoStage = 5;
}
break;

case 5:
drive.TankDrive(0,0,false);
if(LiftSwitch.Get() == true) {
    lift1motor.Set(0);
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    AutoStage = 6;  
    sdfr = true;
    Speed = .4;
}
  else {lift1motor.Set(.4);}
break;


case 6:
if(truepitch <= -4){
AutoStage = 7;
Speed = .7;
sdfr = true;
}
else{
  SpeedDrive();
}
break;


case 7:
if(tip > truepitch){
tip = truepitch;
}
if(truepitch >= tip + 4){ 
  AutoStage = 8;
  sdfr = true;
  Speed = .32;
}
else{
  SpeedDrive();
}
break;


case 8:
if(leftdriveEncoder.GetDistance() >= -6 && rightdriveEncoder.GetDistance() <= 6){
SpeedDrive();
} 
else{
  drive.TankDrive(0,0,false);
  brake.Set(brake.kOff);
  AutoStage = 9;
  tip = truepitch;
}
/*if(truepitch >= tip + 9){
  tip = truepitch;
  AutoStage = 9;
  sdfr = true;
}
else{
  SpeedDrive();
} */
break;


case 9:
if(truepitch >= tip + 11.75){
  brake.Set(brake.kForward);
  AutoStage = 10;
}
else{
  SpeedDrive();
}
break;


case 10:
autoMode = AutoDoNothing;
break;
}
}
break;
//Auto Forward
case AutoForward:
switch (AutoStage){
  case 0:
  if (m_autoSelected == kAutoForward) {
    if (DistanceDrive(-.7,135, true) == DONE) autoactive = false;

  else drive.TankDrive(0,0,false);
}
break;
case 1:
autoMode = AutoDoNothing;
break;
}
break;
case AutoDoNothing: 
      drive.TankDrive(0,0,false); 
    break;


break;
}
}




void Robot::TeleopInit() {
  leftdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
  lift1Encoder.SetDistancePerPulse(1./128.);
}
void Robot::TeleopPeriodic() {


//Drive Modes
  if(rightdrivestick.GetTrigger()) HoldTheLine();
  else if(leftdrivestick.GetTrigger()) StraightDrive();
  else {
    drive.TankDrive((leftdrivestick.GetY() * -1), (rightdrivestick.GetY() * 1), false);
    sdfr = false;}
  if (rightdrivestick.GetRawButton(4)) Abort();



//Arm 1 + Limits
if(gamepad.GetLeftStickButton()){
  lockvalue = -75;
}
else if(gamepad.GetRightStickButton()){
lockvalue = -61;
}
if(gamepad.GetLeftY() > .3 && LiftSwitch.Get()) {
  lift1motor.Set(0);
}
else if(gamepad.GetLeftY() > .3 && !LiftSwitch.Get()) {
  lift1motor.Set(.3);
}
else if(gamepad.GetLeftY() < -.3 && lift1Encoder.GetDistance() <= lockvalue) {
  lift1motor.Set(0);
}
else if(gamepad.GetLeftY() < -.3 && lift1Encoder.GetDistance() > lockvalue) {
lift1motor.Set(gamepad.GetLeftY());
}
else{lift1motor.Set(0);}


//Cone+Cube Wheels
  if (gamepad.GetLeftBumper()) {
    conewheelmotor.Set(1);
    lwheelmotor.Set(.6);
    rwheelmotor.Set(-.6);
  }
  else if (gamepad.GetRightBumper()) {
    conewheelmotor.Set(-1);
    lwheelmotor.Set(-.5);
    rwheelmotor.Set(.5);
  }
  else if(gamepad.GetLeftTriggerAxis() > .7) {
    lwheelmotor.Set(.8);
    rwheelmotor.Set(-.8);
  } 
  else if(gamepad.GetRightTriggerAxis() > .7) {
    lwheelmotor.Set(-.5);
    rwheelmotor.Set(.5);
    }
  else {conewheelmotor.Set(0);
  lwheelmotor.Set(0);
  rwheelmotor.Set(0);
  }
  

//Arm 2
  if(gamepad.GetXButton()) {
    lift2.Set(lift2.kForward);
  }
  else if(gamepad.GetAButton()) {
    lift2.Set(lift2.kReverse);
  }
  else {lift2.Set(lift2.kOff);}

  
//Clamp
  if(gamepad.GetYButton()) {
    clamp.Set(clamp.kForward);
  }
  else if(gamepad.GetBButton()) {
    clamp.Set(clamp.kReverse);
  }
  else {clamp.Set(clamp.kOff);}



//Brake
  if(leftdrivestick.GetRawButton(3)) {
    brake.Set(brake.kForward);
  }
  else if(rightdrivestick.GetRawButton(2)) {
    brake.Set(brake.kReverse);
  }
  else {brake.Set(brake.kOff);}

  

//Mid Lock
 

}
    


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SpeedDrive() {
  if(!sdfr){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    sdfr = true;
  }
  double throttle = Speed;
  double difference = (-1 * rightdriveEncoder.GetDistance()) - (leftdriveEncoder.GetDistance());
  std::cout << "SpeedDrive Difference " << difference << std::endl;
    drive.TankDrive(-1 * (throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }


void Robot::StraightDrive(){
  if (!sdfr){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    sdfr = true;
  }
// Prior year code:
//  double throttle = (-1 *leftdrivestick.GetY());
//  double difference = (-1 * rightdriveEncoder.GetDistance()) - (leftdriveEncoder.GetDistance());
//  drive.TankDrive((throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
// get speed from left stick
  double throttle = (leftdrivestick.GetY());
// calculate difference in left & right encoders
// right encoder reads negative so multiply by -1
// positive difference means right encoder reads higher, so robot has veered left
  double difference = (-1 * rightdriveEncoder.GetDistance()) - (leftdriveEncoder.GetDistance());
// -1 multiplier on left side
  drive.TankDrive(-1 * (throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
  std::cout << "Love isn't always on time" << std::endl; // No I am not ashamed of this TOTO reference
    if (!sdfr){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    sdfr = true;
  }
  drive.TankDrive((-0.05 * leftdriveEncoder.GetDistance()),(-0.05 * rightdriveEncoder.GetDistance()), false);;
}

void Robot::Abort(){
  lift1motor.StopMotor();
  lwheelmotor.StopMotor();
  rwheelmotor.StopMotor();
  conewheelmotor.StopMotor();
  lift2.Set(frc::DoubleSolenoid::Value::kReverse);
  clamp.Set(frc::DoubleSolenoid::Value::kReverse);
  alignarms.Set(frc::DoubleSolenoid::Value::kReverse);
  brake.Set(frc::DoubleSolenoid::Value::kReverse);
  auxSpedCtrlr4DefState = 0;
  auxSpedCtrlr5DefState = 0;
  auxSpedCtrlr6DefState = 0;
  Pnm1DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm2DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm3DefState = frc::DoubleSolenoid::Value::kReverse;
  Pnm4DefState = frc::DoubleSolenoid::Value::kReverse;
}


void Robot::Lock(){
  if (gamepad.GetLeftBumper()) auxSpedCtrlr4DefState = AUXSPDCTL_SPD;
  if (gamepad.GetRightBumper()) auxSpedCtrlr5DefState = AUXSPDCTL_SPD;
  if (rightdrivestick.GetTop()) auxSpedCtrlr6DefState = AUXSPDCTL_SPD;
  if (gamepad.GetXButton()) Pnm1DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetYButton()) Pnm2DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetBButton()) Pnm3DefState = frc::DoubleSolenoid::Value::kForward;
  if (gamepad.GetAButton()) Pnm4DefState = frc::DoubleSolenoid::Value::kForward;
}

int Robot::DistanceDrive (float speed, float distance, bool brake)
{
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
	static float autoStartSpeed;
  static float direction;
	static double lastDistance, speedUpDistance, slowDownDistance;
  static int sameCounter;
  static bool brakingFlag;
  static double brakeStartTime; 

	float newSpeed;
	double curDistance;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    brakingFlag = false;
    FirstCallFlag = false;
    if (speed < 0) {
      direction = -1;
    } else {
      direction = 1;

    }
    autoStartSpeed = AUTOSTARTSPEED;
    if(distance < (DRIVERAMPUPDISTANCE * 2)) {
	    speedUpDistance = distance / 2;
	    slowDownDistance = speedUpDistance;
    } else {
	    speedUpDistance = DRIVERAMPUPDISTANCE;
     	slowDownDistance = distance - DRIVERAMPUPDISTANCE;
    }
	  frc::SmartDashboard::PutNumber(  "DistanceDrive Distance", distance);
  	lastDistance = 0;
    sameCounter = 0;
    leftdriveEncoder.Reset();
  }

 	if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if (((double)AutoTimer.Get() - brakeStartTime) < .2) {
    	drive.TankDrive(-0.2 * direction *FORWARD, -0.2 * direction * FORWARD);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}
  
	curDistance = fabs(leftdriveEncoder.GetDistance());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((fabs(speed) - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = fabs(speed) * (distance-curDistance)/DRIVERAMPUPDISTANCE;
    if (newSpeed < 0.2) newSpeed = 0.2;
	} else {
		newSpeed = fabs(speed);
	}
// temp attempt to drive straight:
  double difference = direction * ((-1 * rightdriveEncoder.GetDistance()) - (leftdriveEncoder.GetDistance()));
// -1 multiplier on left side
 	drive.TankDrive(-1 * (newSpeed + (difference * 0.1)) * direction * AUTOFORWARD, (newSpeed - (difference * 0.1)) * direction * AUTOFORWARD);
// 	drive.TankDrive(-1 * newSpeed * direction * AUTOFORWARD, newSpeed * direction * AUTOFORWARD);


	curDistance = fabs(leftdriveEncoder.GetDistance());
  if (curDistance < distance) {
    return NOTDONEYET;
  } else {
    if (brake) {
      brakingFlag = true;
      brakeStartTime = (double)AutoTimer.Get();
      return NOTDONEYET;
    } else {
      FirstCallFlag = true;
      drive.TankDrive(0, 0);
      return DONE;
    }

  }
  
  // should never get here
  drive.TankDrive(0, 0);
  FirstCallFlag = true;
  return DONE;
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
