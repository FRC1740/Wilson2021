/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/EngageClimberLock.h"

EngageClimberLock::EngageClimberLock(Climber *climber) : m_climber(climber) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_CLIMBER
// Called when the command is initially scheduled.
void EngageClimberLock::Initialize() {
  m_climber->Lock();
}

// Called repeatedly when this Command is scheduled to run
void EngageClimberLock::Execute() {}

// Called once the command ends or is interrupted.
void EngageClimberLock::End(bool interrupted) {}

// Returns true when the command should end.
bool EngageClimberLock::IsFinished() { return true; }
#endif // ENABLE_CLIMBER
