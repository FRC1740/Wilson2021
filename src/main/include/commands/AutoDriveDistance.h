/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDriveDistance
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriveDistance> {
 public:
  explicit AutoDriveDistance(DriveTrain *drivetrain, double distance);

#ifdef ENABLE_DRIVETRAIN
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
#endif // ENABLE_DRIVETRAIN

 private:
  DriveTrain *m_driveTrain;
  double m_distance;
  double m_speedOut = 0.0;
};
