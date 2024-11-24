package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.Commands.pickup
import frc.team449.commands.Commands.readyHigh
import frc.team449.commands.Commands.scoreHighAuto
import frc.team449.commands.Commands.stow

class PreloadHighAndPickup(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to readyHigh(robot),
        1 to stow(robot),
        2 to pickup(robot)
      ),
      stopEventMap = hashMapOf(
        0 to InstantCommand(),
        1 to scoreHighAuto(robot),
        2 to pickup(robot)
      ),
      debug = false,
      timeout = 0.5
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2", "part3"), "high1")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2", "part3"), "high1")
    }
}
