/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/JumbleShooter.h"

JumbleShooter::JumbleShooter(Shooter *shooter, int direction) : m_shooter(shooter), m_direction(direction) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_SHOOTER
// Called when the command is initially scheduled.
void JumbleShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void JumbleShooter::Execute() {
  m_shooter->ForceJumble(m_direction);
}

// Called once the command ends or is interrupted.
void JumbleShooter::End(bool interrupted) {
    m_shooter->Dejumble();
}

// Returns true when the command should end.
bool JumbleShooter::IsFinished() { return false; }
#endif // ENABLE_SHOOTER