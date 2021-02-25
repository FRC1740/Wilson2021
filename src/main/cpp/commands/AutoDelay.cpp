/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoDelay.h"

AutoDelay::AutoDelay(double seconds) : m_seconds(seconds) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_timer = frc::Timer();
}

// Called when the command is initially scheduled.
void AutoDelay::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
//void AutoDelay::Execute() {}

// Called once the command ends or is interrupted.
//void AutoDelay::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDelay::IsFinished() { return m_timer.Get() > m_seconds; }
