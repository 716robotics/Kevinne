// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Approximate distance for auto to drive forward in inches
#define AUTODIST 500

#pragma once
#include <tunables.h>
#include <string>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/MotorControllerGroup.h>
#include <frc/VictorSP.h>
#include <frc/Relay.h>
#include <frc/Servo.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{0};
  frc::Joystick rightDriveStick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  frc::VictorSP lDrive0{0};
  frc::VictorSP lDrive1{1};
  frc::VictorSP rDrive2{2};
  frc::VictorSP rDrive{3};
  frc::MotorControllerGroup lDrive{lDrive0, lDrive1};
  frc::MotorControllerGroup rDrive{rDrive0, rDrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};
  //Effectors
  frc::Compressor compressor;
  frc::VictorSP Lift1Motor4{4};
  frc::VictorSP WheelsMotor5{5};
  frc::VictorSP WheelsMotor6{6};
  frc::VictorSP ConeWheelMotor7{7};
  //frc::VictorSP auxMotorController5{8};
  //frc::VictorSP auxMotorController6{9};
  frc::DoubleSolenoid Lift2Pneumatic{0,1};
  frc::DoubleSolenoid ClampPneumatic{2,3};
  frc::DoubleSolenoid AlignArmsPneumatic{4,5};
  frc::DoubleSolenoid BrakePneumatic{6,7};
  //Encoders
	frc::Encoder leftDriveEncoder{0,1,false,frc::Encoder::k4X};
	frc::Encoder rightDriveEncoder{2,3,false,frc::Encoder::k4X};
  frc::Encoder lift1Encoder{4,5};
  //Global Vars
  frc::Timer AutoTimer;
  bool sdfr = false;
  bool autoactive = true;
  //Digital Inputs
  frc::DigitalInput LiftSwitch {6}; 
  //Default States
  float auxSpedCtrlr4DefState = 0;
  float auxSpedCtrlr5DefState = 0;
  float auxSpedCtrlr6DefState = 0;
  frc::DoubleSolenoid::Value Pnm1DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm2DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm3DefState = frc::DoubleSolenoid::Value::kReverse;
  frc::DoubleSolenoid::Value Pnm4DefState = frc::DoubleSolenoid::Value::kReverse;
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void StraightDrive();
  void HoldTheLine();
  void Abort();
  void Lock();
  int DistanceDrive(float,float,bool);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoMobility = "Score + Back out";
  const std::string kAutoCharge = "Score + Charge"
  const std::string kAutoDoNothing = "Do Nothing";
  std::string m_autoSelected;
};
