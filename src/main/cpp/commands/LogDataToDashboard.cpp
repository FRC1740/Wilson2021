/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LogDataToDashboard.h"
LogDataToDashboard::LogDataToDashboard(Shooter* shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called repeatedly when this Command is scheduled to run
void LogDataToDashboard::Execute() {
#ifdef ENABLE_SHOOTER
  //shooter subsystem
  //  frc::SmartDashboard::PutNumber("Top Motor Speed", shooter->GetTopMotorSpeed());
  //  frc::SmartDashboard::PutNumber("Bottom Motor Speed", shooter->GetBottomMotorSpeed());
#endif // ENABLE_SHOOTER
}
