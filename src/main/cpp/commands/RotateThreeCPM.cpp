/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include "commands/RotateThreeCPM.h"

RotateThreeCPM::RotateThreeCPM(ControlPanelManipulator *controlpanelmanipulator) : m_controlPanelManipulator(controlpanelmanipulator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(controlpanelmanipulator);
#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
  frc::Shuffleboard::SelectTab(ConShuffleboard::ControlPanelManipulatorTab);
  m_rotationCount = m_controlPanelManipulator->m_sbt_CPM->Add("Transition Count", 0).GetEntry();
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
}
#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
// Called when the command is initially scheduled.
void RotateThreeCPM::Initialize() {
  // Read the current color from the smart dashboard
  m_currentColor = m_controlPanelManipulator->ReadCurrentColor();
}

// Called repeatedly when this Command is scheduled to run
void RotateThreeCPM::Execute() {
  m_controlPanelManipulator->Rotate();
  std::string newColor = m_controlPanelManipulator->ReadCurrentColor();

  if (newColor != m_currentColor) {
    m_currentColor = newColor;
    m_rotationCount.SetDouble(++m_transitionCount);
  }
}

// Called once the command ends or is interrupted.
void RotateThreeCPM::End(bool interrupted) {
  m_controlPanelManipulator->Stop();
}

// Returns true when the command should end.
bool RotateThreeCPM::IsFinished() {
  //FIXME: Still need to get the target from the Shuffleboard tab and put the transition count to the Shuffleboard tab
//  unsigned int targetCount = frc::SmartDashboard::GetNumber("Rotation Target", 28);  // Should get from Main Robot Tab
  unsigned int targetCount=28;
  if (m_transitionCount > targetCount) { // HACK: We are UNDERcounting. Catching abt 6 changes per rotation
    m_controlPanelManipulator->Stop();
    m_transitionCount=0;
    return true;
  }
  return false;
}
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
