package frc.team449.commands

import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot

object Commands {

  fun stow(robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.stow(),
        robot.pivot.stow(),
        robot.intake.stop()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() }
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
      robot.intake.hold(),
      robot.elevator.high(),
      robot.pivot.stow()
    )
  }

  fun scoreHighAuto(robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.high(),
        robot.pivot.stow()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
      robot.intake.outtake(),
      WaitCommand(1.0),
      robot.intake.stop()
    )
  }

  fun scoreStack(robot: Robot): Command {
    return SequentialCommandGroup(
      ParallelCommandGroup(
        robot.elevator.stow(),
        robot.pivot.stow()
      ),
      WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
      robot.intake.outtake(),
      WaitCommand(1.0),
      robot.intake.stop()
    )
  }
}
