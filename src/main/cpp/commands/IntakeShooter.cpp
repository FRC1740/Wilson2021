/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeShooter.h"

IntakeShooter::IntakeShooter(Shooter *shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void IntakeShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeShooter::Execute() {
  m_shooter->Index(1);
  m_shooter->Jumble(-1);
}

// Called once the command ends or is interrupted.
void IntakeShooter::End(bool interrupted) {
  m_shooter->Dejumble();
  m_shooter->Undex();
}

// Returns true when the command should end.
bool IntakeShooter::IsFinished() { return false; }
#endif // ENABLE_SHOOTER
