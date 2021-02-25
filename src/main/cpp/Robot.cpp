/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  // Create and get reference to SB tab
  m_sbt_Robot = &frc::Shuffleboard::GetTab(ConShuffleboard::RobotTab);
  // See https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/shuffleboard/layouts-with-code/using-tabs.html

  // Create widget for code version
  #define CODE_VERSION ROBOT_VERSION_STRING " " __DATE__ " " __TIME__ 
  m_nte_CodeVersion = m_sbt_Robot->Add("Code Version", CODE_VERSION).WithSize(3, 1).WithPosition(0, 0).GetEntry();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
   // Ensure that we lock the climber whenever we're disabled
  m_disabledCommand = m_container.GetDisabledCommand();
  
  if (m_disabledCommand != nullptr) {
    m_disabledCommand->Schedule();
  }
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  // Start a match timer... 
  m_lockTimer.Reset();
  m_lockTimer.Start();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // Check the timer
  double time_left = m_lockTimer.Get();
  // If < 2 seconds, Ensure Climber is LOCKED
  if (time_left >= CLIMBER_LOCK_TIME) {
    frc::Shuffleboard::GetTab(ConShuffleboard::RobotTab).Add("Time Left", time_left).WithPosition(0,1).WithSize(1,1);
    // UnlockClimber();
  }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
