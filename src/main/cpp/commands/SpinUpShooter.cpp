/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinUpShooter.h"

SpinUpShooter::SpinUpShooter(Shooter *shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void SpinUpShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SpinUpShooter::Execute() {
  m_shooter->SpinUp();
}

// Called once the command ends or is interrupted.
void SpinUpShooter::End(bool interrupted) {
  m_shooter->StopSpinUp();
}

// Returns true when the command should end.
bool SpinUpShooter::IsFinished() { return false; }
#endif // ENABLE_SHOOTER
