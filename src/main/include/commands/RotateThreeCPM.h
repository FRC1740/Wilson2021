/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ControlPanelManipulator.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RotateThreeCPM
    : public frc2::CommandHelper<frc2::CommandBase, RotateThreeCPM> {
 public:
  explicit RotateThreeCPM(ControlPanelManipulator *controlpanelmanipulator);

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR

 private:
  ControlPanelManipulator *m_controlPanelManipulator;
  nt::NetworkTableEntry m_rotationCount;
  nt::NetworkTableEntry m_detectedColor;
  unsigned int m_transitionCount=0;
  std::string m_currentColor;
};
