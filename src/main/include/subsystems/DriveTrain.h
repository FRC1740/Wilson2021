/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include "Constants.h"

namespace ConDriveTrain {
    // Motors
    constexpr int RIGHT_MOTOR_A_ID = 2;
    constexpr int RIGHT_MOTOR_B_ID = 4;
    constexpr int LEFT_MOTOR_A_ID = 3;
    constexpr int LEFT_MOTOR_B_ID = 5;
    //constexpr double ROTATION_FACTOR = 1/1.3;

    //Spark Max Settings
    constexpr int RAMP_RATE = 0.100; //seconds
    constexpr bool INVERSION = false; //
    //Conversions
    constexpr double IN_2_ENCODER = (6*ConMath::PI)/(42 * 10.71); //encoder to motor 42 counts/rev, motor to shaft 10.71:1, 6in wheel
    // SHould be 6 * pi / (42 * 10.71)
    // Reciprocal (42 * 10.71)/(6 * pi)
    // Seems to scale at abt 40% of expected.
    constexpr double ENCODER_2_IN = 1/IN_2_ENCODER;
    // constexpr double ENCODER_2_IN = (42.0 * 10.71)/(6.0 * ConMath::PI);

    // Conversion factor Ticks -> Inches
    // constexpr double ENCODER_TICKS_TO_INCHES = 2.0 + (2/9); // 0.58
    // constexpr double ENCODER_TICKS_OFFSET = -6.0 - (2/3);
}

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  frc::ShuffleboardTab *m_sbt_DriveTrain;
  nt::NetworkTableEntry m_nte_DriveSpeedFilter;
  nt::NetworkTableEntry m_nte_DriveRotationFilter;
  nt::NetworkTableEntry m_nte_InputExponent;

  nt::NetworkTableEntry m_nte_a_DriveDelay;
  nt::NetworkTableEntry m_nte_b_DriveDistance;
  nt::NetworkTableEntry m_nte_c_ShooterSpinTime;
  nt::NetworkTableEntry m_nte_d_JumblerDelay;
  nt::NetworkTableEntry m_nte_e_JumblerOnTime;

#ifdef ENABLE_DRIVETRAIN
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  //void Periodic();

  /**
   * Drives the robot using arcade controls.
   *
   * @param speed the commanded forward movement
   * @param rotation the commanded rotation
   */
  void ArcadeDrive(double speed, double rotation);

  void TankDrive(double left, double right);

  double GetMaxOutput();

  void SetMaxOutput(double maxOutput);

  double GetRightDistance();

  double GetLeftDistance();

  double GetAverageEncoderDistance();

  void ResetEncoders();
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double m_maxOutput = 1.0;

  // Neo motor controllers
  rev::CANSparkMax m_rightMotorA{ConDriveTrain::RIGHT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotorB{ConDriveTrain::RIGHT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorA{ConDriveTrain::LEFT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorB{ConDriveTrain::LEFT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};

  // Drive encoders
  rev::CANEncoder m_rightEncoderA = m_rightMotorA.GetEncoder();
  rev::CANEncoder m_rightEncoderB = m_rightMotorB.GetEncoder();
  rev::CANEncoder m_leftEncoderA = m_leftMotorA.GetEncoder();
  rev::CANEncoder m_leftEncoderB = m_leftMotorB.GetEncoder();

  // Robot Drive
  frc::DifferentialDrive m_driveTrain{m_leftMotorA, m_rightMotorA};
#endif // ENABLE_DRIVETRAIN
};
