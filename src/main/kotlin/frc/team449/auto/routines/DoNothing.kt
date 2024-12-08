package frc.team449.auto.routines

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import kotlin.jvm.optionals.getOrNull

class DoNothing(
  private val robot: Robot
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive
    )

  override val trajectory: MutableList<ChoreoTrajectory> = mutableListOf()

  override fun createCommand(): Command {
    return InstantCommand({
      if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
        robot.drive.pose = Pose2d(0.0, 0.0, Rotation2d(0.0))
      } else {
        robot.drive.pose = Pose2d(0.0, 0.0, Rotation2d(180.0))
      }
    }).andThen(InstantCommand({ robot.intake.setVoltage(-5.5) }))
      .withTimeout(3.0)
      .andThen(robot.intake.stop())
  }
}
