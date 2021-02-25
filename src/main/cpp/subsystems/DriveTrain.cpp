/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {
#ifdef ENABLE_DRIVETRAIN
  // Settings for Spark Max motor controllers should be done here, in code
  // and not in the Spark Max Client Software
  m_rightMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_rightMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);

  m_rightMotorA.SetInverted(ConDriveTrain::INVERSION);
  m_rightMotorB.SetInverted(ConDriveTrain::INVERSION);
  m_leftMotorA.SetInverted(ConDriveTrain::INVERSION);
  m_leftMotorB.SetInverted(ConDriveTrain::INVERSION);

  // Set additional motor controllers on drive train to follow
  m_rightMotorB.Follow(m_rightMotorA, false);
  m_leftMotorB.Follow(m_leftMotorA, false);
#endif // ENABLE_DRIVETRAIN

  // Create and get reference to SB tab
  m_sbt_DriveTrain = &frc::Shuffleboard::GetTab(ConShuffleboard::DriveTrainTab);

  // Create widgets for digital filter lengths
  m_nte_DriveSpeedFilter    = m_sbt_DriveTrain->AddPersistent("Drive Speed Filter", 10.0)   .WithSize(2, 1).WithPosition(0, 0).GetEntry();
  m_nte_DriveRotationFilter = m_sbt_DriveTrain->AddPersistent("Drive Rotation Filter", 5.0) .WithSize(2, 1).WithPosition(0, 1).GetEntry();

  // Create widget for non-linear input
  m_nte_InputExponent       = m_sbt_DriveTrain->AddPersistent("Input Exponent", 1.0)        .WithSize(1, 1).WithPosition(0, 2).GetEntry();

  // Create widgets for AutoDrive
  m_nte_a_DriveDelay        = m_sbt_DriveTrain->AddPersistent("a Drive Delay", 0.0)         .WithSize(1, 1).WithPosition(1, 0).GetEntry();
  m_nte_b_DriveDistance     = m_sbt_DriveTrain->AddPersistent("b Drive Distance", -20.0)    .WithSize(1, 1).WithPosition(1, 1).GetEntry();
  m_nte_c_ShooterSpinTime   = m_sbt_DriveTrain->AddPersistent("c Shooter Spin Time", 10.0)  .WithSize(1, 1).WithPosition(1, 2).GetEntry();
  m_nte_d_JumblerDelay      = m_sbt_DriveTrain->AddPersistent("d Jumbler Delay ", 5.0)      .WithSize(1, 1).WithPosition(1, 3).GetEntry();
  m_nte_e_JumblerOnTime     = m_sbt_DriveTrain->AddPersistent("e Jumbler On Time", 15.0)    .WithSize(1, 1).WithPosition(1, 4).GetEntry();
}

#ifdef ENABLE_DRIVETRAIN
// This method will be called once per scheduler run
//void DriveTrain::Periodic() {}

// Used by TeleOpDrive
void DriveTrain::ArcadeDrive(double speed, double rotation) {
  m_driveTrain.ArcadeDrive(speed, DeadZone(rotation));
}

// Used by AlignToPlayerStationPID
void DriveTrain::TankDrive(double left, double right){
  m_driveTrain.TankDrive(left, right);
}

// Used by TeleOpSlowDrive
double DriveTrain::GetMaxOutput() {
    return m_maxOutput;
}

void DriveTrain::SetMaxOutput(double maxOutput) {
  m_maxOutput = maxOutput;
  m_driveTrain.SetMaxOutput(maxOutput);
}

// Used by AutoDriveDistance
void DriveTrain::ResetEncoders() {
  m_rightEncoderA.SetPosition(0.0);
  m_rightEncoderB.SetPosition(0.0);
  m_leftEncoderA.SetPosition(0.0);
  m_leftEncoderB.SetPosition(0.0);
}

// FIXME: Account for two encoders per side
double DriveTrain::GetRightDistance() {
  return (m_rightEncoderA.GetPosition() * ConDriveTrain::ENCODER_2_IN);
}

double DriveTrain::GetLeftDistance() {
  return (m_leftEncoderA.GetPosition() * ConDriveTrain::ENCODER_2_IN);
}

// Used by AutoDriveDistance
double DriveTrain::GetAverageEncoderDistance() {
  // return (m_leftEncoderA.GetPosition() - m_rightEncoderA.GetPosition()) / 2.0;
  return ((GetLeftDistance() - GetRightDistance()) / 2.0)/ 13.16; // FIXME: Fudge Factor 
}
#endif // ENABLE_DRIVETRAIN
