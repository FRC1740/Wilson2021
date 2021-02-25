/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RotateManualCPM.h"

RotateManualCPM::RotateManualCPM(ControlPanelManipulator *controlpanelmanipulator, std::function<double()> speed) 
          : m_controlPanelManipulator(controlpanelmanipulator), m_speed(speed) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(controlpanelmanipulator);
}

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
// Called when the command is initially scheduled.
void RotateManualCPM::Initialize() {
  m_controlPanelManipulator->SetSpeed(ConControlPanelManipulator::MOTOR_SPEED);
}

// Called repeatedly when this Command is scheduled to run
void RotateManualCPM::Execute() {
  m_controlPanelManipulator->Rotate();
}

// Called once the command ends or is interrupted.
void RotateManualCPM::End(bool interrupted) {
  m_controlPanelManipulator->Stop();
}

// Returns true when the command should end.
bool RotateManualCPM::IsFinished() { return false; }
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
