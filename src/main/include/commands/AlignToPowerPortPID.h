/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include "subsystems/DriveTrain.h"
#include "subsystems/Vision.h"

class AlignToPowerPortPID
    : public frc2::CommandHelper<frc2::PIDCommand, AlignToPowerPortPID> {
 public:
  AlignToPowerPortPID(Vision *vision, DriveTrain *driveTrain);

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  DriveTrain *m_driveTrain;
  Vision *m_vision;
};
