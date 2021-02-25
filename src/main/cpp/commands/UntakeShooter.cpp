/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/UntakeShooter.h"

UntakeShooter::UntakeShooter(Shooter *shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void UntakeShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void UntakeShooter::Execute() {
  m_shooter->ForceJumble(1);
  m_shooter->ForceIndex(-1);
}

// Called once the command ends or is interrupted.
void UntakeShooter::End(bool interrupted) {
  m_shooter->Undex();
  m_shooter->Dejumble();
}

// Returns true when the command should end.
bool UntakeShooter::IsFinished() { return false; }
#endif // ENABLE_SHOOTER
