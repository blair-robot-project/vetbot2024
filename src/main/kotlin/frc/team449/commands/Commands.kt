package frc.team449.commands

import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.subsystems.elevator.Elevator
import frc.team449.subsystems.intake.Intake
import frc.team449.subsystems.pivot.Pivot

object Commands {
  fun pickup (robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.stow(),
        robot.pivot.stow()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
      robot.intake.intake(),
      WaitCommand(1.0),
      robot.intake.stop()
    )
  }

  fun scoreHigh (robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.high(),
        robot.pivot.high()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
      robot.intake.outtake(),
      WaitCommand(1.0),
      robot.intake.stop()
    )
  }
}