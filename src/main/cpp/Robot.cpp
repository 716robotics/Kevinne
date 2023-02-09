// FRC Team 716 Basic Drive code
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoMobilityCone, kAutoMobilityCone);
  m_chooser.AddOption(kAutoCharge, kAutoCharge);
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  compressor.EnableDigital();

}

void Robot::RobotPeriodic() {

  frc::SmartDashboard::PutNumber("Lift Encoder", lift1Encoder.GetDistance());
  frc::SmartDashboard::PutNumber("Left Wheel Output", power.GetCurrent(5) );  
  frc::SmartDashboard::PutNumber("Right Wheel Output", power.GetCurrent(6) );
  frc::SmartDashboard::PutNumber("Cone Wheel Output", power.GetCurrent(7) ); 
  frc::SmartDashboard::PutBoolean("Limit Swith Value", LiftSwitch.Get());
}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if ( m_autoSelected == kAutoMobilityCone) {autoMode = AutoMobilityCone;}
  else if ( m_autoSelected == kAutoCharge) {autoMode = AutoCharge;}
  else autoMode = AutoDoNothing;
  leftdriveEncoder.Reset();
  rightdriveEncoder.Reset();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoMobilityCone && autoactive) {
    if (DistanceDrive(.7,AUTODIST, true) == DONE) autoactive = false;
  }
  else drive.TankDrive(0,0,false); 

switch (autoMode) {


//Auto Mobility (Cone)
case AutoMobilityCone:
switch (AutoStage) {

case 0:

if(lift1Encoder.GetDistance() < 48) {
lift1motor.Set(.5);
}
else if(lift1Encoder.GetDistance() >= 48) {
  lift1motor.Set(0);
  AutoStage = 1;
  }
else {lift1motor.Set(0);}
break;

case 1:

if(lift2.Get() == lift2.kReverse) {
  lift2.Set(lift2.kForward);
}
else {lift2.Set(lift2.kOff);}

if(lift2.Get() == lift2.kForward) {
  AutoStage = 2;
}
break;

case 2:
if(clamp.Get() == clamp.kForward) {
  clamp.Set(clamp.kReverse);
}
else {clamp.Set(clamp.kOff);}

if(clamp.Get() == clamp.kReverse) {
  AutoStage = 3;

}
break;

case 3:
if(lift2.Get() == lift2.kForward) {
  lift2.Set(lift2.kReverse);
}
else {lift2.Set(lift2.kOff);}

if(lift2.Get() == lift2.kReverse) {
  AutoStage = 4;
}
break;

case 4:
if(LiftSwitch.Get() == true) {
  lift1motor.Set(0);
  lift1Encoder.Reset();
  AutoStage = 5;
}
else {lift1motor.Set(-.5);}
break;

case 5:
break;
}

break;

case AutoMobilityCube:{

}
break;
case AutoCharge:{

}
break;

case AutoDoNothing: {
  
}
break;
}
}




void Robot::TeleopInit() {
  leftdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightdriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
  lift1Encoder.SetDistancePerPulse(1./128.);
}
void Robot::TeleopPeriodic() {
//Encoder call


//Limit Switch

if(LiftSwitch.Get()) {
  lift1Encoder.Reset();

 if(gamepad.GetLeftY() < .3) {
    lift1motor.Set(0);}

else if(gamepad.GetLeftY() > .3) {
    lift1motor.Set(gamepad.GetLeftY());

}


else {lift1motor.Set(0);}
}

//Drive Modes
  if(rightdrivestick.GetTrigger()) HoldTheLine();
  else if(leftdrivestick.GetTrigger()) StraightDrive();
  else {
    drive.TankDrive((leftdrivestick.GetY() * -1), (rightdrivestick.GetY() * 1));
    sdfr = false;}
  if (gamepad.GetBackButtonPressed()) Abort();


//Arm 1
 if(lift1Encoder.GetDistance() >= 22) {
   if(gamepad.GetLeftY() > .3) {
    lift1motor.Set(0);
   }
   else if(gamepad.GetLeftY() < -.3) {
    lift1motor.Set(gamepad.GetLeftY());
   }
   else {lift1motor.Set(0);}
  }
  else if(fabs(gamepad.GetLeftY()) > .3) {
    lift1motor.Set(gamepad.GetLeftY());
    }
  else {lift1motor.Set(0);}


//Cube Wheels
  if(gamepad.GetLeftTriggerAxis() > .5) {
    lwheelmotor.Set(1);
    rwheelmotor.Set(-1);
    }
  else if(gamepad.GetRightTriggerAxis() > .5) {
    lwheelmotor.Set(-1);
    rwheelmotor.Set(1);
  } 
  else {
    lwheelmotor.Set(0);
    rwheelmotor.Set(0);
  }    


//Cone Wheel
  if (gamepad.GetLeftBumper()) {
    conewheelmotor.Set(1);
  }
  else if (gamepad.GetRightBumper()) {
    conewheelmotor.Set(-1);
  }
  else {conewheelmotor.Set(0);}
  

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


//Alignment Arms
  if(leftdrivestick.GetRawButton(2)) {
    alignarms.Set(alignarms.kForward);
  }
  else if(rightdrivestick.GetRawButton(2)) {
    alignarms.Set(alignarms.kReverse);
  }
  else {alignarms.Set(alignarms.kOff);}

//Brake
  if(leftdrivestick.GetRawButton(3)) {
    brake.Set(brake.kForward);
  }
  else if(rightdrivestick.GetRawButton(3)) {
    brake.Set(brake.kReverse);
  }
  else {brake.Set(brake.kOff);}


  

  

 



  }


    


void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(){
  if (!sdfr){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    sdfr = true;
  }
  double throttle = (-1 *leftdrivestick.GetY());
  double difference = (-1 * rightdriveEncoder.GetDistance()) - (leftdriveEncoder.GetDistance());
  drive.TankDrive((throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
  std::cout << "Love isn't always on time" << std::endl; // No I am not ashamed of this TOTO reference
    if (!sdfr){
    leftdriveEncoder.Reset();
    rightdriveEncoder.Reset();
    sdfr = true;
  }
  drive.TankDrive((0.05 * leftdriveEncoder.GetDistance()),(0.05 * rightdriveEncoder.GetDistance()), false);;
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
    autoStartSpeed = direction * AUTOSTARTSPEED;
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
  
	curDistance = abs(leftdriveEncoder.GetDistance());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((speed - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}

	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
	curDistance = abs(leftdriveEncoder.GetDistance());
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
