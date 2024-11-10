package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.Commands.pickup
import frc.team449.commands.Commands.scoreHigh

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
        0 to scoreHigh(robot),
        1 to pickup(robot),
        2 to scoreHigh(robot)
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
