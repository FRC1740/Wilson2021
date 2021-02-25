/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <ctre/Phoenix.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include "Constants.h"

namespace ConControlPanelManipulator {
    // Motor
    constexpr int MOTOR_ID = 3;
    constexpr double MOTOR_SPEED = 810; // 30 RPM x 27:1 = 810 motor RPM, 1 rotation every 2 seconds
    constexpr double DESIRED_TRANSITION_COUNT = 28;
    // PID Constants for TalonSRX
    enum Constants {
        /**
         * Which PID slot to pull gains from.  Starting 2018, you can choose
         * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
         */
        kSlotIdx = 0,

        /* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
        * For now we just want the primary one.
        */
        kPIDLoopIdx = 0,

        /*
        * set to zero to skip waiting for confirmation, set to nonzero to wait
        * and report to DS if action fails.
        */
        kTimeoutMs = 30
    };
}

class ControlPanelManipulator : public frc2::SubsystemBase {
 public:
  ControlPanelManipulator();
  frc::ShuffleboardTab *m_sbt_CPM;
  nt::NetworkTableEntry m_nte_DetectedRed;
  nt::NetworkTableEntry m_nte_DetectedGreen;
  nt::NetworkTableEntry m_nte_DetectedBlue;
  nt::NetworkTableEntry m_nte_MatchedRed;
  nt::NetworkTableEntry m_nte_MatchedGreen;
  nt::NetworkTableEntry m_nte_MatchedBlue;
  nt::NetworkTableEntry m_nte_Confidence;
  nt::NetworkTableEntry m_nte_ColorString;
  nt::NetworkTableEntry m_nte_MotorCurrent;

  nt::NetworkTableEntry m_nte_DesiredTransitions;
  nt::NetworkTableEntry m_nte_ActualTransitions;
  nt::NetworkTableEntry m_nte_RotateMotorSpeed;
  nt::NetworkTableEntry m_nte_GotoMotorSpeed;

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  std::string ReadCurrentColor();
  std::string ReadFieldColor();
  void Rotate();
  void Stop();
  void SetSpeed(double);

 private:
  std::string m_sensedColor;
  std::string m_fieldColor;

  std::string LookupColor(std::string); // Lookup between our sensed color and field color

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  TalonSRX m_rotationMotor {ConControlPanelManipulator::MOTOR_ID}; // 2020 Vendor Library
  double m_currentSpeed;

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};


  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  

  /* Starting REV calibration values */
  /*
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  */

  /* 2020-01-14 Calibrated "LED ON" values */
  static constexpr frc::Color kBlueTarget = frc::Color(0.125, 0.427, 0.449);
  static constexpr frc::Color kGreenTarget = frc::Color(0.166, 0.581, 0.253);
  static constexpr frc::Color kRedTarget = frc::Color(0.523, 0.343, 0.130);
  static constexpr frc::Color kYellowTarget = frc::Color(0.320, 0.559, 0.120);

  // FIXME: Assume Actual FMS Color is at 90 degrees from what we "see"
  /* 
  static constexpr frc::Color kWhenISeeBlue = kRedTarget;
  static constexpr frc::Color kWhenISeeGreen = kYellowTarget;
  static constexpr frc::Color kWhenISeeRed = kBlueTarget;
  static constexpr frc::Color kWhenISeeYellow = kGreenTarget; 
  */
  // FIXME: Straight up same color (NOT FOR COMPETITION - TESTING ONLY)
  static constexpr frc::Color kWhenISeeBlue = kBlueTarget;
  static constexpr frc::Color kWhenISeeGreen = kGreenTarget;
  static constexpr frc::Color kWhenISeeRed = kRedTarget;
  static constexpr frc::Color kWhenISeeYellow = kYellowTarget;
#endif // ENABLE_CONTROL_PANEL_MANIPULATOR
};
