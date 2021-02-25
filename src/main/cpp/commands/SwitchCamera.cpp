/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SwitchCamera.h"

SwitchCamera::SwitchCamera(Vision *vision) : m_vision(vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(vision);
}

#ifdef ENABLE_VISION
// Called when the command is initially scheduled.
void SwitchCamera::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SwitchCamera::Execute() {}

// Called once the command ends or is interrupted.
void SwitchCamera::End(bool interrupted) {}

// Returns true when the command should end.
bool SwitchCamera::IsFinished() { return false; }
#endif // ENABLE_VISION
