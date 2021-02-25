/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FlapHopper.h"

FlapHopper::FlapHopper(Shooter *shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void FlapHopper::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FlapHopper::Execute() {
  m_shooter->FlapHopper();
}

// Called once the command ends or is interrupted.
void FlapHopper::End(bool interrupted) {
  m_shooter->StopFlapper();
}

// Returns true when the command should end.
bool FlapHopper::IsFinished() { return false; }
#endif // ENABLE_SHOOTER