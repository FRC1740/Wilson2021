/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DoubleSolenoid.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>

#include "Constants.h"

namespace ConClimber {
    // Motor
    constexpr int MOTOR_ID = 1; // 9?;
    constexpr int SOLENOID_LOCK_ID = 6; // Test - may have to swap these
    constexpr int SOLENOID_UNLOCK_ID = 7; // Test - may have to swap these
    constexpr double EXT_SPEED = -0.5; // CRE 02-10 Negative motor input should extend
    constexpr double RET_SPEED = 0.5; // CRE 02-10 Positive motor input should retract
    constexpr double ROTATION_DISTANCE = 3.75; // inches #35 Chain = .375 pitch x 10 tooth = 3.75 inches
    constexpr double EXT_LIMIT = -31.0; // Soft limit (inches) to prevent over-extending
    constexpr double RET_LIMIT = -1.6023; // Account for drift - Starting Configuration: fully retracted, Encoder=0
}

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  frc::ShuffleboardTab *m_sbt_Climber;

  nt::NetworkTableEntry m_nte_ClimberDistance;
  nt::NetworkTableEntry m_nte_ClimberSpeed;
  nt::NetworkTableEntry m_nte_Locked;

#ifdef ENABLE_CLIMBER
  // CRE: The driver/codriver controller objects are defined in RobotContainer
  // frc::XboxController codriver_control{ConXBOXControl::CODRIVER_CONTROLLER_PORT};
  frc::XboxController *m_codriver_control = nullptr;

  void ResetEncoder();
 
  void Go(double speed);

  void Stop();

  void Lock();

  void Unlock();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetCodriverControl(frc::XboxController *codriver_control);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //TalonSRX m_motor{ConClimber::MOTOR_ID};
  rev::CANSparkMax m_motor{ConClimber::MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless}; //Replace with SparkMAX
  frc::DutyCycleEncoder m_dutyCycleEncoder{0};
  frc::DoubleSolenoid m_climberLock{ConClimber::SOLENOID_LOCK_ID, ConClimber::SOLENOID_UNLOCK_ID};

  double m_climberPosition;
  bool m_Locked;
  bool m_FirstLock;
  frc::Timer m_timer;
#endif // ENABLE_CLIMBER
};
