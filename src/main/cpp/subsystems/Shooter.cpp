/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Shooter.h"
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/StringMap.h> // for wpi::StringMap
#include <utility> // for std::pair

Shooter::Shooter() {

#ifdef ENABLE_SHOOTER

    m_timer = frc::Timer();
    m_timer.Reset();
    m_timer.Start();
    // Invert shooter motors correctly
    m_topMotor.SetInverted(false);
    m_bottomMotor.SetInverted(true);
    m_kickerMotor.SetInverted(true);
    // m_jumblerMotor.SetInverted(true);

    // Set velocity of shaft relative to velocity of wheel
    m_topEncoder.SetVelocityConversionFactor(ConShooter::Top::VELOCITY_FACTOR);
    m_bottomEncoder.SetVelocityConversionFactor(ConShooter::Bottom::VELOCITY_FACTOR);

    // Set controller gains from constants
    // See this for tuning help (Robot Characterization Tool)
    // https://docs.wpilib.org/en/latest/docs/software/wpilib-overview/new-for-2020.html
    m_topVelocityPID.SetP(ConShooter::Top::P);
    m_topVelocityPID.SetI(ConShooter::Top::I);
    m_topVelocityPID.SetD(ConShooter::Top::D);
    m_topVelocityPID.SetFF(ConShooter::Top::FF);
    m_topVelocityPID.SetOutputRange(0.0, 1.0);
    
    m_bottomVelocityPID.SetP(ConShooter::Bottom::P);
    m_bottomVelocityPID.SetI(ConShooter::Bottom::I);
    m_bottomVelocityPID.SetD(ConShooter::Bottom::D);
    m_bottomVelocityPID.SetFF(ConShooter::Bottom::FF);
    m_bottomVelocityPID.SetOutputRange(0.0, 1.0);

    m_topMotor.BurnFlash();
    m_bottomMotor.BurnFlash();
    
    // First choose kicker sensor
    m_kickerMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 30);    

    // Set the current limit 
    m_kickerMotor.EnableCurrentLimit(true);
    m_kickerMotor.ConfigContinuousCurrentLimit(ConShooter::Kicker::CONT_CURRENT_LIMIT); 
    m_kickerMotor.ConfigPeakCurrentLimit(ConShooter::Kicker::PEAK_CURRENT_LIMIT);
    m_kickerMotor.ConfigPeakCurrentDuration(ConShooter::Kicker::PEAK_CURRENT_DURATION);

    // Set the peak and nominal outputs
    m_kickerMotor.ConfigNominalOutputForward(0, 30);
    m_kickerMotor.ConfigNominalOutputReverse(0, 30);
    m_kickerMotor.ConfigPeakOutputForward(1, 30);
    m_kickerMotor.ConfigPeakOutputReverse(-1, 30);

    // Kicker motor PID code
    m_kickerMotor.SelectProfileSlot(0,0);
    m_kickerMotor.Config_kP(0, ConShooter::Kicker::P, 30); // "Slot", Value, timeout (ms)
    m_kickerMotor.Config_kI(0, ConShooter::Kicker::I, 30);  
    m_kickerMotor.Config_kD(0, ConShooter::Kicker::D, 30); 
    m_kickerMotor.Config_kF(0, ConShooter::Kicker::F, 30);
    
    m_kickerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    m_jumblerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_hopperFlapper.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
#endif // ENABLE_SHOOTER

  // Create and get reference to SB tab
  m_sbt_Shooter = &frc::Shuffleboard::GetTab(ConShuffleboard::ShooterTab);

  m_nte_TopMotorInputRPM = m_sbt_Shooter->
    AddPersistent("Top Motor Input RPM", ConShooter::Top::OPTIMAL_RPM)
    .WithSize(2, 1)
    .WithPosition(0, 0)
    .GetEntry();

  m_nte_BottomMotorInputRPM = m_sbt_Shooter->
    AddPersistent("Bottom Motor Input RPM", ConShooter::Bottom::OPTIMAL_RPM)
    .WithSize(2, 1)
    .WithPosition(0, 1)
    .GetEntry();

  m_nte_TopMotorOutputRPM = m_sbt_Shooter->
    AddPersistent("Top Motor Output RPM", 0.0)
    .WithWidget(frc::BuiltInWidgets::kGraph)
    .WithSize(6, 2)
    .WithPosition(2, 0)
    .GetEntry();

  m_nte_BottomMotorOutputRPM = m_sbt_Shooter->
    AddPersistent("Bottom Motor Output RPM", 0.0)
    .WithWidget(frc::BuiltInWidgets::kGraph)
    .WithSize(6, 2)
    .WithPosition(2, 2)
    .GetEntry();

  m_nte_EnableMotorGraphs = m_sbt_Shooter->
    AddPersistent("Enable Motor Graphs", true)
    .WithWidget(frc::BuiltInWidgets::kToggleButton)
    .WithSize(1, 1)
    .WithPosition(8, 0)
    .GetEntry();

  m_nte_JumblerMotorSpeed = m_sbt_Shooter->
    AddPersistent("Jumbler Motor Speed", ConShooter::Jumbler::MOTOR_SPEED)
    .WithSize(2, 1)
    .WithPosition(0, 3)
    .GetEntry();

  m_nte_KickerInputRPM = m_sbt_Shooter->
    AddPersistent("Kicker RPM", ConShooter::Kicker::OPTIMAL_RPM)
    .WithSize(1, 1)
    .WithPosition(0, 2)
    .GetEntry();

  m_nte_KickerMotorError = m_sbt_Shooter->
    AddPersistent("Kicker Error", 0.0)
    .WithSize(2,1)
    .WithPosition(6,1)
    .GetEntry();

  /* Is there a way to get the current speed of a TalonSRX Motor? */
  m_nte_KickerMotorVoltage = m_sbt_Shooter->
    AddPersistent("Kicker Motor Voltage", 0.0)
    .WithSize(2, 1)
    .WithPosition(6, 2)
    .GetEntry();

  m_nte_IndexSensorOutput = m_sbt_Shooter->
    AddPersistent("Index Sensor Output", 0.0)
    .WithSize(2,1)
    .WithPosition(8, 1)
    .GetEntry();

  m_nte_LoadSensorOutput = m_sbt_Shooter->
    AddPersistent("Load Sensor Output", 0.0)
    .WithSize(1, 1)
    .WithPosition(8, 2)
    .GetEntry();

  m_nte_LoadSensorDistance = m_sbt_Shooter->
    AddPersistent("Load Sensor Distance", 0.0)
    .WithSize(1,1)
    .WithPosition(9,2)
    .GetEntry();

  m_nte_IntakeDelay = m_sbt_Shooter->
    AddPersistent("Intake delay", 0.5)
    .WithSize(1,1)
    .WithPosition(3,4)
    .GetEntry();

  m_nte_DesiredIntakeSpeed = m_sbt_Shooter->
    AddPersistent("Desired Intake Speed", 0.0)
    .WithSize(1,1)
    .WithPosition(0,4)
    .GetEntry();
  
  m_nte_ActualIntakeSpeed = m_sbt_Shooter->
    AddPersistent("Actual Intake Speed", 0.0)
    .WithSize(1,1)
    .WithPosition(1,4)
    .GetEntry();

  m_nte_ShooterDelay = m_sbt_Shooter->
    AddPersistent("Shooter Delay", 0.0)
    .WithSize(1,1)
    .WithPosition(8,5)
    .GetEntry();

  /*
  m_nte_JumblerStatus = m_sbt_Shooter->
    AddPersistent("Jumbler Status", false)
    .WithWidget(frc::BuiltInWidgets::kToggleButton)
    .WithSize(2, 1)
    .WithPosition(3, 2)
    .GetEntry();
  */
}

#ifdef ENABLE_SHOOTER
// This method will be called once per scheduler run
void Shooter::Periodic() {

    // Update Network Table/Shuffleboard Values
    m_nte_TopMotorOutputRPM.SetDouble(GetTopMotorSpeed());
    m_nte_BottomMotorOutputRPM.SetDouble(GetBottomMotorSpeed());
    m_nte_KickerMotorVoltage.SetDouble(GetKickerMotorVoltage());
    m_nte_KickerMotorError.SetDouble(GetKickerError());
    m_nte_IndexSensorOutput.SetDouble(m_IndexSensor.GetRange());
    m_nte_LoadSensorOutput.SetDouble(m_LoadSensor.GetAverageVoltage());
    m_nte_LoadSensorDistance.SetDouble((m_LoadSensor.GetAverageVoltage() / ConShooter::Loader::VOLTAGE_TO_IN) + 1.7);
    m_nte_ActualIntakeSpeed.SetDouble(0.0); //FIXME:: Make this return the Load Motor's speed

#if 0 // Test code for the sensors
    // Check TimeofFLight sensor to see if a powerCell is ... stuck? loaded? ??
    frc::SmartDashboard::PutNumber("Range: ", m_IndexSensor.GetRange());
    // Greater than 30 b/c if no object is sensed it returns between 0-1, right in front of the sensor returns ~40
    if (m_IndexSensor.GetRange() < 300.0 && m_IndexSensor.GetRange() > 30.0) { // FIXME: range in mm 
        frc::SmartDashboard::PutBoolean("PowerCell", true);
        m_indexMotor.Set(TalonSRXControlMode::PercentOutput, ConShooter::Indexer::MOTOR_SPEED);
    }
    else {
        frc::SmartDashboard::PutBoolean("PowerCell", false);
        m_indexMotor.Set(TalonSRXControlMode::PercentOutput, 0.0);
    } 

    if (m_LoadSensor.GetAverageVoltage() < 2.0) {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, ConShooter::Loader::MOTOR_SPEED);
    }
    else {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, 0);
    }
#endif
}


// Used internally only
void Shooter::SetBottomMotorSpeed(double velocity) {
    double vlimit = (velocity > ConShooter::Bottom::MAX_RPM) ? ConShooter::Bottom::MAX_RPM : velocity;
    m_bottomVelocityPID.SetReference(vlimit, rev::ControlType::kVelocity);
}

// Used internally only
void Shooter::SetTopMotorSpeed(double velocity) {
    double vlimit = (velocity > ConShooter::Top::MAX_RPM) ? ConShooter::Top::MAX_RPM : velocity;
    m_topVelocityPID.SetReference(vlimit, rev::ControlType::kVelocity);
}

// Used internally only for dashboard
double Shooter::GetBottomMotorSpeed() {
    return m_bottomEncoder.GetVelocity();
}

// Used internally only for dashboard
double Shooter::GetTopMotorSpeed() {
    return m_topEncoder.GetVelocity();
}

// Used internally only for dashboard
double Shooter::GetKickerMotorVoltage() {
    // Showing current draw might be more useful?
    // return m_kickerMotor.GetOutputCurrent();
    return m_kickerMotor.GetMotorOutputVoltage();
}

// Used internally only for dashboard
double Shooter::GetKickerError() {
    return m_kickerMotor.GetClosedLoopError();
}

// Used by SpinUpShooter
void Shooter::SpinUp()
{
  double bottomMotorTarget = m_nte_BottomMotorInputRPM.GetDouble(ConShooter::Bottom::OPTIMAL_RPM);

#if 0 // Implement Fine-Tuning of bottom shooter motor
  if (m_codriver_control != nullptr) {
    bottomMotorTarget += ConShooter::Bottom::RPM_FINE_TUNE * m_codriver_control->GetRawAxis(ConLaunchPad::Dial::RIGHT); 
  }
#endif

  SetTopMotorSpeed(m_nte_TopMotorInputRPM.GetDouble(ConShooter::Top::OPTIMAL_RPM));
  SetBottomMotorSpeed(bottomMotorTarget);
  // FIXME: CRE Why are we setting a value for the "kicker" motor in the SpinUp() method?
  SetKickerSpeed(m_nte_KickerMotorSpeed.GetDouble(ConShooter::Kicker::OPTIMAL_RPM));
}

// Used by SpinUpShooter
void Shooter::StopSpinUp(){
  //SetTopMotorSpeed(0.0);
  //SetBottomMotorSpeed(0.0);
  m_topVelocityPID.SetReference(0.0, rev::ControlType::kVoltage);
  m_bottomVelocityPID.SetReference(0.0, rev::ControlType::kVoltage);
  SetKickerSpeed(0.0);
}

// Used by JumbleShooter
void Shooter::Jumble(int direction) {
  double speed = m_nte_JumblerMotorSpeed.GetDouble(ConShooter::Jumbler::MOTOR_SPEED);
  if (IsIndexSensorClear()) {
    if (direction != 1) { speed = -speed; }
    m_jumblerMotor.Set(TalonSRXControlMode::PercentOutput, speed);
    m_hopperFlapper.Set(TalonSRXControlMode::PercentOutput, ConShooter::HopperFlapper::MOTOR_SPEED);
  }
  else {
    // I think we want to stop the tower/loader belts here which is hopperFlapper
    m_jumblerMotor.Set(TalonSRXControlMode::PercentOutput, 0);
  }
}

void Shooter::ForceJumble(int direction) {
  double speed = m_nte_JumblerMotorSpeed.GetDouble(ConShooter::Jumbler::MOTOR_SPEED);
  if (direction != 1) { speed = -speed; }
  m_jumblerMotor.Set(TalonSRXControlMode::PercentOutput, speed);
  m_hopperFlapper.Set(TalonSRXControlMode::PercentOutput, ConShooter::HopperFlapper::MOTOR_SPEED);
}

// Used by JumbleShooter
void Shooter::Dejumble() {
  m_jumblerMotor.Set(TalonSRXControlMode::PercentOutput, 0.0);
  m_hopperFlapper.Set(TalonSRXControlMode::PercentOutput, 0.0);
}

// "Choo choo" motor used by FlapHopper() for manual control
void Shooter::FlapHopper() {
  m_hopperFlapper.Set(TalonSRXControlMode::PercentOutput, ConShooter::HopperFlapper::MOTOR_SPEED);
}

// "Choo choo" motor used by FlapHopper() for manual control
void Shooter::StopFlapper() {
  m_hopperFlapper.Set(TalonSRXControlMode::PercentOutput, 0.0);  
}

// Used internally only
void Shooter::SetKickerSpeed(double speed) {
  // Power Mode...
  // m_kickerMotor.Set(TalonSRXControlMode::PercentOutput, speed);
  /// Velocity Mode for use with hex shaft encoder
  m_kickerMotor.Set(TalonSRXControlMode::Velocity, speed); 
}

// Used by RobotContainer to bring controller object into subsystem
// Periodic will check for reset of encoder
void Shooter::SetCodriverControl(frc::XboxController *codriver_control) {
  m_codriver_control = codriver_control;
}

void Shooter::Index(int direction) {
#if 0 // Turned off to test TOF sensor as the index sensor
    if (m_IndexSensor.GetRange() < 300.0 && m_IndexSensor.GetRange() > 30.0) {
        m_loadMotor.Set(TalonSRXControlMode::Velocity, 0);
        m_nte_DesiredIntakeSpeed.SetDouble(0.0);
        m_lastIntake = (m_timer.Get() - m_nte_IntakeDelay.GetDouble(0.0));
    }
    else if (m_LoadSensor.GetAverageVoltage() < 2.0) { //FIXME: Change
        m_loadMotor.Set(TalonSRXControlMode::Velocity, (ConShooter::Loader::MOTOR_SPEED * direction));
        m_lastIntake = m_timer.Get();
        m_nte_DesiredIntakeSpeed.SetDouble(800.0);
    }
    else if (m_lastIntake + m_nte_IntakeDelay.GetDouble(0.0) < m_timer.Get()) {
        m_loadMotor.Set(TalonSRXControlMode::Velocity, 0);
        m_nte_DesiredIntakeSpeed.SetDouble(0.0);
    }
#endif

#if 0
    if (m_IndexSensor.GetRange() < 100.0 ) {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, (m_nte_DesiredIntakeSpeed.GetDouble(0.1)));
        m_lastIntake = m_timer.Get();
    }
    else if ((m_lastIntake + m_nte_IntakeDelay.GetDouble(0.0) < m_timer.Get())) {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, 0);
    }
#endif

#if 0
    if (m_IndexSensor.GetRange() < 100.0) {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, m_nte_DesiredIntakeSpeed.GetDouble(0.1));
    }
    else {
        m_loadMotor.Set(TalonSRXControlMode::PercentOutput, 0);
    }
#endif
}

void Shooter::Undex() {
  m_loadMotor.Set(TalonSRXControlMode::Velocity, 0);
  //m_nte_DesiredIntakeSpeed.SetDouble(0.0);
}

// FIXME: Comments, please!! What is the intention of this method?
void Shooter::ForceIndex(int direction) {
  m_loadMotor.Set(TalonSRXControlMode::Velocity, 800 * direction);
  m_nte_DesiredIntakeSpeed.SetDouble(800.0 * direction);
}

bool Shooter::IsIndexSensorClear() {
  return (m_IndexSensor.GetRange() > 300.0 || m_IndexSensor.GetRange() < 30.0);
}

double Shooter::ShooterDelay() {
  return m_nte_ShooterDelay.GetDouble(0.0);
}
#endif // ENABLE_SHOOTER
