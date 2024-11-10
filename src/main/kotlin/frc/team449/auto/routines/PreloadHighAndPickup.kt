package frc.team449.auto.routines

import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.Commands.pickup
import frc.team449.commands.Commands.readyHigh
import frc.team449.commands.Commands.scoreHigh
import frc.team449.commands.Commands.stow
import frc.team449.commands.Commands.stowAndHold

class PreloadHighAndPickup(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to readyHigh(robot),
        1 to pickup(robot)
      ),
      stopEventMap = hashMapOf(
        0 to scoreHigh(robot),
        1 to stow(robot)
          .withTimeout(1.0)
          .andThen(pickup(robot)),
        2 to stowAndHold(robot)
      ),
      debug = false,
      timeout = 0.25
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
