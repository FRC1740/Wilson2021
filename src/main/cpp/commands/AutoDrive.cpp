/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoDrive.h"
#include "commands/AutoDriveDistance.h"
#include "commands/AutoDriveAngle.h"
#include "commands/AutoShoot.h"
#include "commands/AutoDelay.h"
#include "commands/SpinUpShooter.h"
#include "commands/JumbleShooter.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

/*
From Kyle:
3 independent commands that are in parallel, not series:

(1) Delay a seconds, drive backwards for b seconds
(2) spin up shooter, delay c seconds, turn off shooter
(3) delay d seconds, turn jumbler for e seconds, turn off jumbler

That would allow for shoot then scoot or scoot then shoot by changing delays and durations, as required by our reliable minimum shooting range.
*/

AutoDrive::AutoDrive(DriveTrain *drivetrain, Shooter *shooter) : m_driveTrain(drivetrain) {
#if defined(ENABLE_DRIVETRAIN) && defined(ENABLE_SHOOTER)
  double a = m_driveTrain->m_nte_a_DriveDelay.GetDouble(0.0); // Drive delay
  double b = m_driveTrain->m_nte_b_DriveDistance.GetDouble(-20.0); // Drive distance (inches)
  double c = m_driveTrain->m_nte_c_ShooterSpinTime.GetDouble(10.0); // Shooter spin time
  double d = m_driveTrain->m_nte_d_JumblerDelay.GetDouble(5.0); // Jumbler delay
  double e = m_driveTrain->m_nte_e_JumblerOnTime.GetDouble(15.0); // Jumbler on time

  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands (
    frc2::SequentialCommandGroup{ AutoDelay(a), 
                                  frc2::ParallelRaceGroup{ AutoDriveDistance(drivetrain, b), AutoDelay(5.0) }
                                },
    frc2::ParallelRaceGroup{ SpinUpShooter(shooter), AutoDelay(c) },
    frc2::SequentialCommandGroup{ AutoDelay(d),
                                  frc2::ParallelRaceGroup{ JumbleShooter(shooter, -1), AutoDelay(e) }
                                }
  );
#endif // defined(ENABLE_DRIVETRAIN) && defined(ENABLE_SHOOTER)
}
