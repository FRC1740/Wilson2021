/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Climber.h"

Climber::Climber() {
    m_sbt_Climber = &frc::Shuffleboard::GetTab(ConShuffleboard::ClimberTab);

    m_nte_ClimberDistance = m_sbt_Climber->AddPersistent("Climber Position", 0.0)
          .WithSize(2,1)
          .WithPosition(0,0)
          .GetEntry();
    m_nte_ClimberSpeed = m_sbt_Climber->AddPersistent("Climber Speed", 0.0)
          .WithSize(2,1)
          .WithPosition(0,2)
          .GetEntry();
    m_nte_Locked = m_sbt_Climber->AddPersistent("Chain Locked?", true)
          .WithSize(1,1)
          .WithPosition(2,0)
          .WithWidget(frc::BuiltInWidgets::kBooleanBox)
          .GetEntry();

#ifdef ENABLE_CLIMBER
    // Set Encoder distance per rotation
    m_dutyCycleEncoder.SetDistancePerRotation(ConClimber::ROTATION_DISTANCE);
    m_dutyCycleEncoder.Reset();
    m_climberPosition = m_dutyCycleEncoder.GetDistance();
    m_motor.BurnFlash();
    Unlock(); // Starting Configuration: Engage chain lock
    /*
    // FIXME: is it better to keep our own timer or use the GetMatchTime() function?
    m_timer = frc::Timer();
    m_timer.Start();
    */
#endif // ENABLE_CLIMBER
}

#ifdef ENABLE_CLIMBER

// Used internally only
void Climber::ResetEncoder() {
  m_dutyCycleEncoder.Reset();
}

// Used only by OperateManualClimber
void Climber::Go(double speed) {
  // If lock is engaged, OR we're beyond the encoder soft limits, DON'T RUN
  if (m_Locked) { 
    m_motor.Set(0.0);
  }

  // If Green Switch is active (Up/On), IGNORE encoder limits *** DANGER ****
  else if ((m_codriver_control != nullptr) && 
      (m_codriver_control->GetRawButton(ConLaunchPad::Switch::GREEN))) { // Nearest to climber controls
      m_motor.Set(speed);
  }
  // Extend = Decreasing/More Negative
  else if (speed < 0.0 && m_climberPosition > ConClimber::EXT_LIMIT) {
    printf("Extending...\n");
    m_motor.Set(speed);
  }
  // Retract = Increasing/More Positive
  else if (speed > 0.0 && m_climberPosition < ConClimber::RET_LIMIT) {
    printf("Retracting...\n");
    m_motor.Set(speed);
  }
  else {
    m_motor.Set(0.0); // Fail Safe
  }
}

// Used only by OperateManualClimber
void Climber::Stop() {
  m_motor.Set(0.0);
}

// Used by OperateManualClimber and EngageClimberLock
void Climber::Lock() {
  m_Locked = true;
  m_motor.Set(0.0);
  m_climberLock.Set(frc::DoubleSolenoid::kForward);
}

// Used only by OperateManualClimber
void Climber::Unlock() {
  m_climberLock.Set(frc::DoubleSolenoid::kReverse);
  m_Locked = false;
}

// This method will be called once per scheduler run
void Climber::Periodic() {
  m_climberPosition = m_dutyCycleEncoder.GetDistance();
  m_nte_ClimberDistance.SetDouble(m_climberPosition);
  m_nte_ClimberSpeed.SetDouble(m_motor.Get());
  m_nte_Locked.SetBoolean(m_Locked);

  if ((!m_Locked) &&
     (m_codriver_control != nullptr)) {
    Climber::Go(m_codriver_control->GetRawAxis(ConLaunchPad::RIGHT_STICK_Y));
  }

  if ((m_codriver_control != nullptr) && 
      (m_codriver_control->GetRawButton(ConLaunchPad::Switch::GREEN))) { // Nearest to climber controls
    ResetEncoder();
  }
  /*
  // FIXME: Determine which of these to do
  if (m_timer.GetMatchTime() < 1) {
    Climber::Lock();
  }
  if (m_timer.HasPeriodPassed(145)) {
    Climber::Lock();
  }
  */
}

// Used by RobotContainer to bring controller object into subsystem
// Periodic will check for reset of encoder
void Climber::SetCodriverControl(frc::XboxController *codriver_control) {
  m_codriver_control = codriver_control;
}
#endif // ENABLE_CLIMBER
