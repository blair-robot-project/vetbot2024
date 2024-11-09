package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.Robot
import frc.team449.auto.AutoUtil

class HighOne(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to InstantCommand(),
        1 to InstantCommand()
      ),
      stopEventMap = hashMapOf(
        0 to SequentialCommandGroup(
          ParallelCommandGroup(
            robot.elevator.stow(),
            robot.pivot.stow()
          ),
          WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
          robot.intake.intake(),
          WaitCommand(1.0),
          robot.intake.stop()
        ),
        1 to SequentialCommandGroup(
          ParallelCommandGroup(
            robot.elevator.high(),
            robot.pivot.high()
          ),
          WaitUntilCommand { robot.elevator.atSetpoint() && robot.pivot.atSetpoint() },
          robot.intake.outtake(),
          WaitCommand(1.0),
          robot.intake.stop()
        )
      ),
      debug = false,
      timeout = 1.5
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "high1")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "high1")
    }
}
