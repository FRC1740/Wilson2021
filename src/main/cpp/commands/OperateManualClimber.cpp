/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/OperateManualClimber.h"
#include "subsystems/Climber.h"
#include "Constants.h"

OperateManualClimber::OperateManualClimber(Climber *climber) : m_climber(climber) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

#ifdef ENABLE_CLIMBER
// Called when the command is initially scheduled.
void OperateManualClimber::Initialize() {
  // m_climber->Unlock();
}

// Called repeatedly when this Command is scheduled to run
void OperateManualClimber::Execute() {
  // CRE 10/14/21 The joysticks on the LaunchPad controller are NOT working
  // double speed = m_climber->m_codriver_control->GetRawAxis(ConLaunchPad::RIGHT_STICK_Y); // Inverted
  // double speed = -m_climber->m_codriver_control->GetRawAxis(ConLaunchPad::LEFT_STICK_Y); // Inverted
  double speed = m_climber->m_driver_control->GetRawAxis(ConXBOXControl::RIGHT_JOYSTICK_Y); // Inverted
  m_climber->Go(speed);
}

// Called once the command ends or is interrupted.
void OperateManualClimber::End(bool interrupted) {
  // m_climber->Stop();
  // m_climber->Lock();
}

// Returns true when the command should end.
bool OperateManualClimber::IsFinished() { return false; }
#endif // ENABLE_CLIMBER
