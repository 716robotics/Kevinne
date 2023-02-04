// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Approximate distance for auto to drive forward in inches


#pragma once
#include <tunables.h>
#include <string>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/VictorSP.h>
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
  frc::Joystick leftdrivestick{0};
  frc::Joystick rightdrivestick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  frc::VictorSP ldrive0{0};
  frc::VictorSP ldrive1{1};
  frc::VictorSP rdrive0{2};
  frc::VictorSP rdrive1{3};
  frc::MotorControllerGroup lDrive{ldrive0, ldrive1};
  frc::MotorControllerGroup rDrive{rdrive0, rdrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};
  //Effectors
  frc::Compressor compressor{frc::PneumaticsModuleType::CTREPCM};
  frc::VictorSP lift1motor{4};
  frc::VictorSP lwheelmotor{5};
  frc::VictorSP rwheelmotor{6};
  frc::VictorSP conewheelmotor{7};
  //frc::VictorSP auxMotorController5{8};
  //frc::VictorSP auxMotorController6{9};
  frc::DoubleSolenoid lift2{frc::PneumaticsModuleType::CTREPCM, 0, 1,};
  frc::DoubleSolenoid clamp{frc::PneumaticsModuleType::CTREPCM, 2, 3,};
  frc::DoubleSolenoid alignarms{frc::PneumaticsModuleType::CTREPCM, 4, 5,};
  frc::DoubleSolenoid brake{frc::PneumaticsModuleType::CTREPCM, 6, 7,};
  //Encoders
	frc::Encoder leftdriveEncoder{0,1,false,frc::Encoder::k4X};
	frc::Encoder rightdriveEncoder{2,3,false,frc::Encoder::k4X};
  frc::Encoder lift1Encoder{4,5,false,frc::Encoder::k4X};
  //Global Vars
  frc::Timer AutoTimer;
  bool sdfr = false;
  bool autoactive = true;
  enum autoModeTypes {AutoMobilityCone, AutoCharge, AutoMobilityCube, AutoDoNothing,} autoMode;
  int AutoStage = 0;
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
  const std::string kAutoMobilityCone = "Score Cone + Back out";
  const std::string kAutoMobilityCube = "Score Cube + Back out";
  const std::string kAutoCharge = "Score + Charge";
  const std::string kAutoDoNothing = "Do Nothing";
  std::string m_autoSelected;
};
