/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AlignToPlayerStationPID.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AlignToPlayerStationPID::AlignToPlayerStationPID(Vision *vision, DriveTrain *driveTrain)
    : CommandHelper(frc2::PIDController(
      ConVision::AlignToPlayerStation::P,
      ConVision::AlignToPlayerStation::I,
      ConVision::AlignToPlayerStation::D),
#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
                    // This should return the measurement
                    [vision,this] {
                      double P = vision->m_nte_Align_P.GetDouble(0.0);
                      double I = vision->m_nte_Align_I.GetDouble(0.0);
                      double D = vision->m_nte_Align_D.GetDouble(0.0);
                      GetController().SetP(P);
                      GetController().SetI(I);
                      GetController().SetD(D);
                      return vision->Align(); 
                    },
                    // This should return the setpoint (can also be a constant)
                    [] { return 0.0; },
                    // This uses the output
                    [driveTrain](double output) {
                      // Use the output here
                      driveTrain->TankDrive(-output, output);
                    },
#else // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
    [] { return 0.0; },
    [] { return 0.0; },
    [](double output) { },
#endif // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
                    { driveTrain, vision }
                    ) {
#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
                      m_vision = vision;
                      m_driveTrain = driveTrain;
                      m_vision->SelectPlayerStationPipeline();
                      m_vision->LightOn();
#endif
                    }
                    
#if defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
// Returns true when the command should end.



bool AlignToPlayerStationPID::IsFinished() { return GetController().AtSetpoint(); }

void AlignToPlayerStationPID::End(bool interrupted) {
  m_vision->LightOff();
  m_driveTrain->TankDrive(0, 0);
}
#else // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
bool AlignToPlayerStationPID::IsFinished() { return true; }

void AlignToPlayerStationPID::End(bool interrupted) {}

#endif // defined(ENABLE_VISION) && defined(ENABLE_DRIVETRAIN)
