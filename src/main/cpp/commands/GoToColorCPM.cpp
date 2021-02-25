/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GoToColorCPM.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <string>
#include <cctype> // for tolower (forces string to be a certain case)

GoToColorCPM::GoToColorCPM(ControlPanelManipulator *controlpanelmanipulator) : m_controlPanelManipulator(controlpanelmanipulator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(controlpanelmanipulator);
}

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
// Called when the command is initially scheduled.
void GoToColorCPM::Initialize() {
 // Read the target color from Network Tables
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("FMSInfo");
  m_targetColor = table->GetString("GameSpecificMessage", "PURPLE");
  frc::SmartDashboard::PutString("Desired Color: ", m_targetColor);
}

// Called repeatedly when this Command is scheduled to run
void GoToColorCPM::Execute() {
  // Rotate the ControlPanel
  m_controlPanelManipulator->Rotate();
}

// Called once the command ends or is interrupted.
void GoToColorCPM::End(bool interrupted) {
  m_controlPanelManipulator->Stop();
}

bool iequals(const std::string& a, const std::string& b) {
  size_t sz = a.size();
  if (sz != b.size())
      return false;
  for (size_t i=0; i<sz; ++i)
      if (std::tolower(a[i]) != std::tolower(b[i]))
          return false;
  return true;
}
// Returns true when the command should end.
bool GoToColorCPM::IsFinished() { 
  // if ColorScan = targetcolor return true;
  // Scan for current color
//  std::string detectedColor = frc::SmartDashboard::GetString("Detected Color", "Orange");
  std::string fieldColor = m_controlPanelManipulator->ReadFieldColor();
  if (iequals(fieldColor, m_targetColor)) {
      m_controlPanelManipulator->Stop();
      return true;    
  }

  return false;
}
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
