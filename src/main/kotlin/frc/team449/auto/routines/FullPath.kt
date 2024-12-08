package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.Commands.pickup
import frc.team449.commands.Commands.scoreStack
import frc.team449.commands.Commands.stowAndHold

class FullPath(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to WaitCommand(1.28).andThen(pickup(robot)),
        1 to stowAndHold(robot),
        2 to WaitCommand(1.28).andThen(pickup(robot)),
        3 to stowAndHold(robot),
        4 to WaitCommand(1.28).andThen(pickup(robot))
      ),
      stopEventMap = hashMapOf(
        0 to scoreStack(robot).withTimeout(0.25),
        2 to scoreStack(robot).withTimeout(0.25),
        4 to scoreStack(robot).withTimeout(0.25),
      ),
      debug = false,
      timeout = 0.75
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("S3-B4", "B4 - S2", "S2 - B3", "B3 - S1", "S1 - B2"), "FullPath")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("S3-B4", "B4 - S2", "S2 - B3", "B3 - S1", "S1 - B2"), "FullPath")
    }
}
