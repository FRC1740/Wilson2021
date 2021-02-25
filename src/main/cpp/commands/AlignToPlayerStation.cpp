/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AlignToPlayerStation.h"

AlignToPlayerStation::AlignToPlayerStation(Vision *vision, DriveTrain *driveTrain) : m_vision(vision), m_driveTrain(driveTrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(vision);
  AddRequirements(driveTrain);
}

#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
// Called when the command is initially scheduled.
void AlignToPlayerStation::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignToPlayerStation::Execute() {
  double rotation_factor = m_vision->Align();
  m_driveTrain->TankDrive(rotation_factor, -rotation_factor);
}

// Called once the command ends or is interrupted.
void AlignToPlayerStation::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignToPlayerStation::IsFinished() { return false; }
#endif // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
