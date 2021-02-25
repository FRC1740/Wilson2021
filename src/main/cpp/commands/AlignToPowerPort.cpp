/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AlignToPowerPort.h"

AlignToPowerPort::AlignToPowerPort(Vision *vision) : m_vision(vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(vision);
}

#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
// Called when the command is initially scheduled.
void AlignToPowerPort::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignToPowerPort::Execute() {}

// Called once the command ends or is interrupted.
void AlignToPowerPort::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignToPowerPort::IsFinished() { return false; }
#endif // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
