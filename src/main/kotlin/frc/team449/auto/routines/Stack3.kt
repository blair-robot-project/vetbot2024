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

class Stack3(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to WaitCommand(1.28).andThen(pickup(robot)),
        1 to stowAndHold(robot)
      ),
      stopEventMap = hashMapOf(
        0 to scoreStack(robot),
        2 to scoreStack(robot)
      ),
      debug = false,
      timeout = 0.25
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("S3-B1", "B1-S2"), "Stack3")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("S3-B1", "B1-S2"), "Stack3")
    }
}
