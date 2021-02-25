/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ControlPanelManipulator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <stdio.h>

ControlPanelManipulator::ControlPanelManipulator() {
    m_sbt_CPM = &frc::Shuffleboard::GetTab(ConShuffleboard::ControlPanelManipulatorTab);

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
    // Optimize rotation speed for RotateThree command, scale down for GoToColor
    m_currentSpeed = ConControlPanelManipulator::MOTOR_SPEED; 
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);


		m_rotationMotor.ConfigFactoryDefault();
        /* first choose the sensor */
		m_rotationMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.SetSensorPhase(true);

		/* set the peak and nominal outputs */
		m_rotationMotor.ConfigNominalOutputForward(0, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.ConfigNominalOutputReverse(0, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.ConfigPeakOutputForward(1, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.ConfigPeakOutputReverse(-1, ConControlPanelManipulator::kTimeoutMs);
		/* set closed loop gains in slot0 */
		m_rotationMotor.Config_kF(ConControlPanelManipulator::kPIDLoopIdx, 0.1097, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.Config_kP(ConControlPanelManipulator::kPIDLoopIdx, 0.22, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.Config_kI(ConControlPanelManipulator::kPIDLoopIdx, 0.0, ConControlPanelManipulator::kTimeoutMs);
		m_rotationMotor.Config_kD(ConControlPanelManipulator::kPIDLoopIdx, 0.0, ConControlPanelManipulator::kTimeoutMs);

    // Shuffleboard Tab Network Table Entries
    m_nte_DetectedRed   = m_sbt_CPM->AddPersistent("Red",            0.0).WithSize(1, 1).WithPosition(0, 0).GetEntry();
    m_nte_DetectedGreen = m_sbt_CPM->AddPersistent("Green",          0.0).WithSize(1, 1).WithPosition(0, 1).GetEntry();
    m_nte_DetectedBlue  = m_sbt_CPM->AddPersistent("Blue",           0.0).WithSize(1, 1).WithPosition(0, 2).GetEntry();
    m_nte_MatchedRed    = m_sbt_CPM->AddPersistent("Matched R",      0.0).WithSize(1, 1).WithPosition(1, 0).GetEntry();
    m_nte_MatchedGreen  = m_sbt_CPM->AddPersistent("Matched G",      0.0).WithSize(1, 1).WithPosition(1, 1).GetEntry();
    m_nte_MatchedBlue   = m_sbt_CPM->AddPersistent("Matched B",      0.0).WithSize(1, 1).WithPosition(1, 2).GetEntry();
    m_nte_Confidence    = m_sbt_CPM->AddPersistent("Confidence",     0.0).WithSize(1, 1).WithPosition(2, 0).GetEntry();
    m_nte_ColorString   = m_sbt_CPM->AddPersistent("Detected Color", "P").WithSize(1, 1).WithPosition(2, 1).GetEntry();
    m_nte_MotorCurrent  = m_sbt_CPM->AddPersistent("Motor Current",  0.0).WithSize(1, 1).WithPosition(2, 2).GetEntry();

    m_nte_ActualTransitions  = m_sbt_CPM->AddPersistent("Actual Transitions", 0.0).WithSize(2, 1).WithPosition(4, 0).GetEntry();
    m_nte_DesiredTransitions = m_sbt_CPM->AddPersistent("Desired Transitions", ConControlPanelManipulator::DESIRED_TRANSITION_COUNT)
                                                                                   .WithSize(2, 1).WithPosition(6, 0).GetEntry();
    m_nte_RotateMotorSpeed   = m_sbt_CPM->AddPersistent("Rotate Motor Speed",  ConControlPanelManipulator::MOTOR_SPEED)
                                                                                   .WithSize(2, 1).WithPosition(6, 1).GetEntry();
    m_nte_GotoMotorSpeed     = m_sbt_CPM->AddPersistent("GoTo Motor Speed",    ConControlPanelManipulator::MOTOR_SPEED/2.0)
                                                                                   .WithSize(2, 1).WithPosition(6, 2).GetEntry();
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
}

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
// This method will be called once per scheduler run
void ControlPanelManipulator::Periodic() {

  /**
   * The method GetColor() returns a normalized color value from the sensor and can be
   * useful if outputting the color to an RGB LED or similar. To
   * read the raw color, use GetRawColor().
   * 
   * The color sensor works best when within a few inches from an object in
   * well lit conditions (the built in LED is a big help here!). The farther
   * an object is the more light from the surroundings will bleed into the 
   * measurements and make it difficult to accurately determine its color.
   */
  frc::Color detectedColor = m_colorSensor.GetColor();
  // frc::Shuffleboard::SelectTab(ConShuffleboard::ControlPanelManipulatorTab);

  /**
   * Run the color match algorithm on our detected color
   */
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);


  // FIXME: change these to : if (matchedColor == kWhenISeeBlue) , etc.
  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
  } else if (matchedColor == kRedTarget) {
    colorString = "Red";
  } else if (matchedColor == kGreenTarget) {
    colorString = "Green";
  } else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
  } else {
    colorString = "Unknown";
  }
  /* */
  m_sensedColor = colorString;
  m_fieldColor = LookupColor(m_sensedColor);
  // Open Smart Dashboard to see the color detected by the sensor.

  m_nte_DetectedRed.  SetDouble(detectedColor.red);
  m_nte_DetectedGreen.SetDouble(detectedColor.green);
  m_nte_DetectedBlue. SetDouble(detectedColor.blue);

  m_nte_MatchedRed.   SetDouble(matchedColor.red);
  m_nte_MatchedGreen. SetDouble(matchedColor.green);
  m_nte_MatchedBlue.  SetDouble(matchedColor.blue);
  
  m_nte_Confidence.   SetDouble(confidence);
  m_nte_ColorString.  SetString(colorString);
  m_nte_MotorCurrent. SetDouble(m_rotationMotor.GetOutputCurrent());
}

std::string ControlPanelManipulator::ReadCurrentColor() {
  return m_sensedColor;
}
std::string ControlPanelManipulator::ReadFieldColor() {
  return m_fieldColor;
}

std::string ControlPanelManipulator::LookupColor(std::string source) { // FIXME: Design color lookup scheme
  return source;
}

void ControlPanelManipulator::Rotate() {
  m_rotationMotor.Set(ControlMode::Velocity, m_currentSpeed); // Maybe 300 RPM?
}

void ControlPanelManipulator::SetSpeed(double speed) {
  m_currentSpeed = speed;
}

void ControlPanelManipulator::Stop() {
  m_rotationMotor.Set(ControlMode::Velocity, 0.0);
  m_currentSpeed = ConControlPanelManipulator::MOTOR_SPEED;
}
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
