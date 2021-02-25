/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DisengageClimberLock.h"

DisengageClimberLock::DisengageClimberLock(Climber *climber) : m_climber(climber) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_CLIMBER
// Called when the command is initially scheduled.
void DisengageClimberLock::Initialize() {
  m_climber->Unlock();
}

// Called repeatedly when this Command is scheduled to run
//void DisengageClimberLock::Execute() {}

// Called once the command ends or is interrupted.
//void DisengageClimberLock::End(bool interrupted) {}

// Returns true when the command should end.
bool DisengageClimberLock::IsFinished() { return true; }
#endif // ENABLE_CLIMBER
