/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FireShooter.h"

FireShooter::FireShooter(Shooter *shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_shooter);
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void FireShooter::Initialize() {
  m_timerStart = m_shooter->m_timer.Get();
}

// Called repeatedly when this Command is scheduled to run
void FireShooter::Execute() {
  m_shooter->ForceIndex(1);

  if (m_shooter->m_timer.Get() - m_timerStart > m_shooter->ShooterDelay()) {
    m_shooter->ForceJumble(-1);
  }
}

// Called once the command ends or is interrupted.
void FireShooter::End(bool interrupted) {
  m_shooter->Undex();
  m_shooter->Dejumble();
}

// Returns true when the command should end.
bool FireShooter::IsFinished() { return false; }
#endif // ENABLE_SHOOTER
