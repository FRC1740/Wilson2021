/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Button.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/AutoDrive.h"
#include "commands/TeleOpDrive.h"
#include "commands/TeleOpSlowDrive.h"
#include "commands/OperateManualClimber.h"
#include "commands/SpinUpShooter.h"
#include "commands/AlignToPlayerStationPID.h"
#include "commands/AlignToPowerPortPID.h"
#include "commands/SwitchCamera.h"
#include "commands/ToggleVisionLight.h"
#include "commands/RotateThreeCPM.h"
#include "commands/GoToColorCPM.h"
#include "commands/RotateManualCPM.h"
#include "commands/JumbleShooter.h"
#include "commands/LogDataToDashboard.h" 
#include "commands/AutoDriveDistance.h"
#include "commands/FlapHopper.h"
#include "commands/IntakeShooter.h"
#include "commands/UntakeShooter.h"
#include "commands/FireShooter.h"
#include "commands/EngageClimberLock.h"
#include "commands/DisengageClimberLock.h"

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autoDrive(&m_driveTrain, &m_shooter), m_lockClimber(&m_climber) {
  // ANOTHER WAY OF CONSTRUCTING: m_autoDriveDistance = AutoDriveDistance(&m_driveTrain);
  
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

#ifdef ENABLE_DRIVETRAIN
  // Set up default drive command
  m_driveTrain.SetDefaultCommand(TeleOpDrive(
    &m_driveTrain,
    [this] { return driver_control.GetRawAxis(ConXBOXControl::RIGHT_TRIGGER) - driver_control.GetRawAxis(ConXBOXControl::LEFT_TRIGGER); },
    [this] { return driver_control.GetRawAxis(ConXBOXControl::LEFT_JOYSTICK_X); }));
#endif // ENABLE_DRIVETRAIN

#ifdef ENABLE_CLIMBER
  // Make climber aware of operator input
  m_climber.SetCodriverControl(&codriver_control);
#endif // ENABLE_CLIMBER

#ifdef ENABLE_SHOOTER
  // Make shooter aware of operator input
  m_shooter.SetCodriverControl(&codriver_control);
#endif // ENABLE_SHOOTER

}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

#if 0 // Testing AutoDriveDistance
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::A); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, 10.0), false);
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::B); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, -10.0), false);
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::X); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, 100.0), false);
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::Y); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, -100.0), false);
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::SELECT); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, 1000.0), false);
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::START); }).WhenPressed(new AutoDriveDistance(&m_driveTrain, -1000.0), false);
#endif

#ifdef ENABLE_DRIVETRAIN
  // Commence reduced speed driving when bumper(s) pressed
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::RIGHT_BUMPER); }).WhenHeld(new TeleOpSlowDrive(&m_driveTrain));
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::LEFT_BUMPER); }).WhenHeld(new TeleOpSlowDrive(&m_driveTrain));
#endif // ENABLE_DRIVETRAIN

#ifdef ENABLE_CLIMBER
  // Climber
  // FIXME April: Two White
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::WHITE); }).WhenReleased(new DisengageClimberLock(&m_climber));
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::WHITE); }).WhenPressed(new EngageClimberLock(&m_climber));
#endif // ENABLE_CLIMBER

#ifdef ENABLE_SHOOTER
  // Shooter
  // FIXME April: Two Red/Blue/Yellow
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Switch::RED); }).WhenHeld(new SpinUpShooter(&m_shooter));
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Switch::BLUE); }).WhenHeld(new IntakeShooter(&m_shooter));
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Switch::YELLOW); }).WhileHeld(new FlapHopper(&m_shooter));

  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::RED); }).WhileHeld(new FireShooter(&m_shooter));
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::BLUE); }).WhileHeld(new UntakeShooter(&m_shooter));
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::YELLOW); }).WhileHeld(new JumbleShooter(&m_shooter, 1));
#endif // ENABLE_SHOOTER

#ifdef ENABLE_VISION
  // Vision
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::SELECT); }).WhileHeld(new AlignToPlayerStationPID(&m_vision, &m_driveTrain));
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::START); }).WhileHeld(new AlignToPowerPortPID(&m_vision, &m_driveTrain));
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::X); }).WhenHeld(new SwitchCamera(&m_vision));
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::Y); }).WhenPressed(new ToggleVisionLight(&m_vision));
#endif // ENABLE_VISION

#ifdef ENABLE_CONTROL_PANEL_MANIPULATOR
  // ControlPanelManipulator
  // FIXME: change RotateThreeCPM and GoToColorCPM to be on the LaunchPad
  // frc2::Button([this] {return codriver_control.GetRawButton(ConXBOXControl::B); }).WhenPressed(new GoToColorCPM(&m_controlPanelManipulator), false);

  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::GREEN); }).WhenPressed(new RotateThreeCPM(&m_controlPanelManipulator), false);
  frc2::Button([this] {return codriver_control.GetRawButton(ConLaunchPad::Button::YELLOW); }).WhenPressed(new GoToColorCPM(&m_controlPanelManipulator), false);


  // FIXME: Convert to the Launchpad? 
  /*
  Cannot call a command directly
  RotateManualCPM( &m_controlPanelManipulator,
    [this] { return codriver_control.GetRawAxis(ConXBOXControl::RIGHT_TRIGGER) - codriver_control.GetRawAxis(ConXBOXControl::LEFT_TRIGGER); } );

  RotateManualCPM( &m_controlPanelManipulator,
    [this] { return codriver_control.GetRawAxis(ConLaunchPad::RIGHT_STICK_X); } ); */

#endif // ENABLE_CONTROL_PANEL_MANIPULATOR

#if 0
  // FIXME: This does not mave a button assigned- remove?
  frc2::Button([this] { return true; }).WhileHeld(new LogDataToDashboard(&m_shooter));
#endif

  /*
  ACCORDING TO DOCS.WPILIB:

  WhenPressed - schedules a command when a trigger/button changes from inactive to active; 
      command will not be scheduled again unless the trigger/button becomes inactive and then active again

  WhenHeld - schedules a command when a trigger/button changes from inactive to active, 
      and cancels it when the trigger/button becomes inactive again. The command will *not* be 
      re-scheduled if it finishes while the trigger/button is still active

  WhileHeld - schedules a command repeatedly while a trigger/button is active, and
      cancels it when the trigger/button becomes inactive again. The command *will* be 
      re-scheduled if it finishes while the trigger/button is still active

  WhenReleased - schedules a command when a trigger/button changes from active to inactive;
      command will not be re-scheduled unless the trigger/button becomes active and then inactive again

  NOTE: these can have multiple parameters, including "interruptable" which is default true
  */
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Command will be run in autonomous
  return &m_autoDrive;
}

frc2::Command* RobotContainer::GetDisabledCommand() {
  // Command will be run in disabled mode
  return &m_lockClimber;
}
