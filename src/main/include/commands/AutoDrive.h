/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"

class AutoDrive
    : public frc2::CommandHelper<frc2::ParallelCommandGroup, AutoDrive> {
 public:
  explicit AutoDrive(DriveTrain *drivetrain, Shooter *shooter);

 private:
  DriveTrain *m_driveTrain;
};
