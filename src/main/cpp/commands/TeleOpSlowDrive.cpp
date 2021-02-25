/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TeleOpSlowDrive.h"

TeleOpSlowDrive::TeleOpSlowDrive(DriveTrain *drivetrain) : m_driveTrain(drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
}

#ifdef ENABLE_DRIVETRAIN
// Called when the command is initially scheduled.
void TeleOpSlowDrive::Initialize() {
  double max_output = m_driveTrain->GetMaxOutput() * 0.5;
  m_driveTrain->SetMaxOutput(max_output);
}

// Called once the command ends or is interrupted.
void TeleOpSlowDrive::End(bool interrupted) {
  double max_output = m_driveTrain->GetMaxOutput() * 2.0;
  m_driveTrain->SetMaxOutput(max_output);
}
#endif // ENABLE_DRIVETRAIN
