package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.Commands.scoreHighAuto

class Preload(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to InstantCommand()
      ),
      stopEventMap = hashMapOf(
        0 to scoreHighAuto(robot),
        1 to PrintCommand("Finished the 1st trajectory!")
      ),
      debug = false,
      timeout = 1.5
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("part1"), "taxi")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("part1"), "taxi")
    }
}
