// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignCrossHair.h"

AlignCrossHair::AlignCrossHair(DriveTrain *drive, LimeLight *light) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_light = light;
  m_drive = drive;
}

// Called when the command is initially scheduled.
void AlignCrossHair::Initialize() {
  m_light->SetLEDMode(ConLimelight::LED_ON);
  //Delay(0.200);
  double angle = m_light->GetHorizontalOffset();
  // FIXME: The SetSafety() method does not exist in the drive train subsystem
  // m_drive->SetSafety(false);
  m_drive->ResetEncoders();
  // FIXME: The GoToAngle() method does not exist in the drive train subsystem
  // m_drive->GoToAngle(angle);
}
// Called repeatedly when this Command is scheduled to run
void AlignCrossHair::Execute() {}

// Called once the command ends or is interrupted.
void AlignCrossHair::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignCrossHair::IsFinished() {
  return false;
}
