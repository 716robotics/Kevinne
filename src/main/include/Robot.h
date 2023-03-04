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
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include <frc/PowerDistribution.h>

 #include <wpi/sendable/Sendable.h>
 #include <wpi/sendable/SendableHelper.h>
 #include "frc/I2C.h"
 #include "frc/SPI.h"
 #include "frc/SerialPort.h"
 #include "frc/Timer.h"
 #include "frc/interfaces/Gyro.h"
 #include "ITimestampedDataSubscriber.h"
 #include "networktables/NetworkTableEntry.h"
 #include <hal/SimDevice.h>

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
  frc::PowerDistribution power{0, frc::PowerDistribution::ModuleType::kCTRE};
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
  enum autoModeTypes {AutoMobilityCone, AutoChargeCone, AutoMobilityCube, AutoChargeCube, AutoForward, AutoDoNothing,} autoMode;
  int AutoStage = 0;
  int AutoMini = 0;
  //Digital Inputs
  frc::DigitalInput LiftSwitch {9};
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
  void SpeedDrive();
  void StraightDrive();
  void HoldTheLine();
  void Abort();
  void Lock();
  bool AutoBalance();
  int DistanceDrive(float,float,bool);
  float m_autodistance;
  float lockvalue = 0;
  bool lockbool = false;
  double Speed; 
  double tip;
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoMobilityCone = "Score Cone + Back out";
  const std::string kAutoMobilityCube = "Score Cube + Back out";
  const std::string kAutoChargeCone = "Score Cone + Charge";
  const std::string kAutoChargeCube = "Score Cube + Charge";
  const std::string kAutoForward = "Drive Forward";
  const std::string kAutoDoNothing = "Do Nothing";
  std::string m_autoSelected;
};

 /*
  * AHRS.h
  *
  *  Created on: Jul 30, 2015
  *      Author: Scott
  */
  
 #ifndef SRC_AHRS_H_
 #define SRC_AHRS_H_
  
  
 class IIOProvider;
 class ContinuousAngleTracker;
 class InertialDataIntegrator;
 class OffsetTracker;
 class AHRSInternal;
  
 class AHRS : public frc::Gyro,
              public wpi::Sendable,
              public wpi::SendableHelper<AHRS>  {
 public:
  
     enum BoardAxis {
         kBoardAxisX = 0,
         kBoardAxisY = 1,
         kBoardAxisZ = 2,
     };
  
     struct BoardYawAxis
     {
         BoardAxis board_axis;
         bool up;
     };
  
     enum SerialDataType {
     kProcessedData = 0,
     kRawData = 1
     };
  
 private:
     friend class AHRSInternal;
     AHRSInternal *      ahrs_internal;
  
     volatile float      yaw;
     volatile float      pitch;
     volatile float      roll;
     volatile float      compass_heading;
     volatile float      world_linear_accel_x;
     volatile float      world_linear_accel_y;
     volatile float      world_linear_accel_z;
     volatile float      mpu_temp_c;
     volatile float      fused_heading;
     volatile float      altitude;
     volatile float      baro_pressure;
     volatile bool       is_moving;
     volatile bool       is_rotating;
     volatile float      baro_sensor_temp_c;
     volatile bool       altitude_valid;
     volatile bool       is_magnetometer_calibrated;
     volatile bool       magnetic_disturbance;
     volatile float      quaternionW;
     volatile float      quaternionX;
     volatile float      quaternionY;
     volatile float      quaternionZ;
  
     /* Integrated Data */
     float velocity[3];
     float displacement[3];
  
  
     /* Raw Data */
     volatile int16_t    raw_gyro_x;
     volatile int16_t    raw_gyro_y;
     volatile int16_t    raw_gyro_z;
     volatile int16_t    raw_accel_x;
     volatile int16_t    raw_accel_y;
     volatile int16_t    raw_accel_z;
     volatile int16_t    cal_mag_x;
     volatile int16_t    cal_mag_y;
     volatile int16_t    cal_mag_z;
  
     /* Configuration/Status */
     volatile uint8_t    update_rate_hz;
     volatile int16_t    accel_fsr_g;
     volatile int16_t    gyro_fsr_dps;
     volatile int16_t    capability_flags;
     volatile uint8_t    op_status;
     volatile int16_t    sensor_status;
     volatile uint8_t    cal_status;
     volatile uint8_t    selftest_status;
  
     /* Board ID */
     volatile uint8_t    board_type;
     volatile uint8_t    hw_rev;
     volatile uint8_t    fw_ver_major;
     volatile uint8_t    fw_ver_minor;
  
     long                last_sensor_timestamp;
     double              last_update_time;
  
     InertialDataIntegrator *integrator;
     ContinuousAngleTracker *yaw_angle_tracker;
     OffsetTracker *         yaw_offset_tracker;
     IIOProvider *           io;
  
     std::thread *           task;
  
     // Simulation
     hal::SimDevice m_simDevice;
  
 #define MAX_NUM_CALLBACKS 3
     ITimestampedDataSubscriber *callbacks[MAX_NUM_CALLBACKS];
     void *callback_contexts[MAX_NUM_CALLBACKS];
     
     bool enable_boardlevel_yawreset;
     double last_yawreset_request_timestamp;
     double last_yawreset_while_calibrating_request_timestamp;
     uint32_t successive_suppressed_yawreset_request_count;
     bool disconnect_startupcalibration_recovery_pending;
     bool logging_enabled;
  
 public:
     AHRS(frc::SPI::Port spi_port_id);
     AHRS(frc::I2C::Port i2c_port_id);
     AHRS(frc::SerialPort::Port serial_port_id);
  
     AHRS(frc::SPI::Port spi_port_id, uint8_t update_rate_hz);
     AHRS(frc::SPI::Port spi_port_id, uint32_t spi_bitrate, uint8_t update_rate_hz);
  
     AHRS(frc::I2C::Port i2c_port_id, uint8_t update_rate_hz);
  
     AHRS(frc::SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
  
     float  GetPitch();
     float  GetRoll();
     float  GetYaw();
     float  GetCompassHeading();
     void   ZeroYaw();
     bool   IsCalibrating();
     bool   IsConnected();
     double GetByteCount();
     double GetUpdateCount();
     long   GetLastSensorTimestamp();
     float  GetWorldLinearAccelX();
     float  GetWorldLinearAccelY();
     float  GetWorldLinearAccelZ();
     bool   IsMoving();
     bool   IsRotating();
     float  GetBarometricPressure();
     float  GetAltitude();
     bool   IsAltitudeValid();
     float  GetFusedHeading();
     bool   IsMagneticDisturbance();
     bool   IsMagnetometerCalibrated();
     float  GetQuaternionW();
     float  GetQuaternionX();
     float  GetQuaternionY();
     float  GetQuaternionZ();
     void   ResetDisplacement();
     void   UpdateDisplacement( float accel_x_g, float accel_y_g,
                                int update_rate_hz, bool is_moving );
     float  GetVelocityX();
     float  GetVelocityY();
     float  GetVelocityZ();
     float  GetDisplacementX();
     float  GetDisplacementY();
     float  GetDisplacementZ();
     double GetAngle() const override;
     double GetRate() const override;
     void   SetAngleAdjustment(double angle);
     double GetAngleAdjustment();
     void   Reset() override;
     float  GetRawGyroX();
     float  GetRawGyroY();
     float  GetRawGyroZ();
     float  GetRawAccelX();
     float  GetRawAccelY();
     float  GetRawAccelZ();
     float  GetRawMagX();
     float  GetRawMagY();
     float  GetRawMagZ();
     float  GetPressure();
     float  GetTempC();
     AHRS::BoardYawAxis GetBoardYawAxis();
     std::string GetFirmwareVersion();
  
     bool RegisterCallback( ITimestampedDataSubscriber *callback, void *callback_context);
     bool DeregisterCallback( ITimestampedDataSubscriber *callback );
  
     int GetActualUpdateRate();
     int GetRequestedUpdateRate();
  
     void EnableLogging(bool enable);
     void EnableBoardlevelYawReset(bool enable);
     bool IsBoardlevelYawResetEnabled();
  
     int16_t GetGyroFullScaleRangeDPS();
     int16_t GetAccelFullScaleRangeG();
  
     void Calibrate() override;
  
 private:
     void SPIInit( frc::SPI::Port spi_port_id, uint32_t bitrate, uint8_t update_rate_hz );
     void I2CInit( frc::I2C::Port i2c_port_id, uint8_t update_rate_hz );
     void SerialInit(frc::SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
     void commonInit( uint8_t update_rate_hz );
     static int ThreadFunc(IIOProvider *io_provider);
  
     /* SendableBase implementation */
     void InitSendable(wpi::SendableBuilder& builder) override;
  
     uint8_t GetActualUpdateRateInternal(uint8_t update_rate);
  
     nt::NetworkTableEntry m_valueEntry;
 };
  
 #endif /* SRC_AHRS_H_ */
AHRS gyro{frc::SPI::Port::kMXP};