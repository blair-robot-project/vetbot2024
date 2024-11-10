package frc.team449.commands

import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot

object Commands {

  fun stow(robot: Robot): Command {
    return ParallelCommandGroup(
      robot.elevator.stow(),
      robot.pivot.stow(),
      robot.intake.stop()
    )
  }

  fun stowAndHold(robot: Robot): Command {
    return ParallelCommandGroup(
      robot.elevator.stow(),
      robot.pivot.stow(),
      robot.intake.hold()
    )
  }

  fun pickup(robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.stow(),
        robot.pivot.intakeAngle()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
      robot.intake.intake()
    )
  }

  fun readyHigh(robot: Robot): Command {
    return ParallelCommandGroup(
      robot.elevator.high(),
      robot.pivot.high()
    )
  }

  fun scoreHigh(robot: Robot): Command {
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
