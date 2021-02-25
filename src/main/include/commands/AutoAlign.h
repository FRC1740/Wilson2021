/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include "subsystems/Vision.h"
#include "subsystems/DriveTrain.h"

class AutoAlign
    : public frc2::CommandHelper<frc2::PIDCommand, AutoAlign> {
 public:
  explicit AutoAlign(DriveTrain *driveTrain, Vision *vision);

#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
  bool IsFinished() override;

  void End(bool interrupted) override;
#endif // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)

 private:
  DriveTrain *m_driveTrain;
  Vision *m_vision;
};
