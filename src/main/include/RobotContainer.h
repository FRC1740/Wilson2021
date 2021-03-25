/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#include "commands/EngageClimberLock.h"

/* Aren't the following  all included in RobotContainer.cpp ? */
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc/XboxController.h>

#include "Constants.h"

#include "subsystems/DriveTrain.h"
#include "subsystems/Climber.h"
#include "subsystems/Shooter.h"
#include "subsystems/Jumbler.h"
// #include "subsystems/ControlPanelManipulator.h"
#include "subsystems/Limelight.h"
#include "subsystems/Vision.h"

#include "commands/AutoDrive.h"
#include "commands/EngageClimberLock.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  frc2::Command* GetDisabledCommand();

 private:
  // The robot's subsystems and commands are defined here...
  DriveTrain m_driveTrain;
  Climber m_climber;
  Shooter m_shooter;
  Jumbler m_jumbler;
//  ControlPanelManipulator m_controlPanelManipulator;
  Vision m_vision;
  LimeLight m_light;
  // Commands that are defined in this scope
  AutoDrive m_autoDrive;
  EngageClimberLock m_lockClimber;

  /*
  EXAMPLE:
  int m_x;
  string m_name; 

  WHERE:
  int and string are types
  m_x and m_name are members of the RobotContainer

  SO:
  DriveTrain m_driveTrain;
  DriveTrain is the type
  m_driveTrain is the member of RobotContainer
  */

public:
  // The driver's game controller
  frc::XboxController driver_control{ConXBOXControl::DRIVER_CONTROLLER_PORT};
  // The codriver's game controller
  frc::XboxController codriver_control{ConLaunchPad::LAUNCHPAD_CONTROLLER_PORT};

  void ConfigureButtonBindings();
};
