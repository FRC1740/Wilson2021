// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include "Constants.h"


class LimeLight : public frc2::SubsystemBase {
 public:

  frc::ShuffleboardTab *m_sbt_Vision;
  std::shared_ptr<nt::NetworkTable> m_Limelight_Table;
  nt::NetworkTableEntry *m_nte_Limelight;
  nt::NetworkTableEntry *m_nte_Align_P;
  nt::NetworkTableEntry *m_nte_Align_I;
  nt::NetworkTableEntry *m_nte_Align_D;

  double m_nte_tx;

  LimeLight();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  double GetValidTarget();

  double GetHorizontalOffset();

  double GetVerticalOffset();

  double GetTargetArea();

  double GetTargetDistance();

  void SetLEDMode(int mode);

  void SetCamMode(int mode);

  void SetSnapshot(int mode);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
