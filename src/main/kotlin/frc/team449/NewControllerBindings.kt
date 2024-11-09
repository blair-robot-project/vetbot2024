package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.subsystems.drive.holonomic.swerve.SwerveSim
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.elevator.ElevatorConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class NewControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  private fun robotBindings() {
    /** Call robot functions you create below */
    //pivotChar()
//    pointToRight()
    raiseArm()
    lowerArm()
    raiseElevator()
    lowerElevator()
//    elevatorChar()
  }

  private fun nonRobotBindings() {
    // slowDrive()

    resetGyro()

    // addNoiseToSimulatedPose()
  }

  private fun slowDrive() {
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )
  }

  private fun resetGyro() {
    driveController.start().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.drive.heading = Rotation2d(PI) }),
        InstantCommand({ robot.drive.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )
  }

  private fun addNoiseToSimulatedPose() {
    driveController.b().onTrue(
      ConditionalCommand(
        InstantCommand({
          robot.drive as SwerveSim
          robot.drive.resetPos()
        }),
        InstantCommand()
      ) { RobotBase.isSimulation() }
    )
  }

  private fun pointToRight() {
    driveController.a().onTrue(
      robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
    )
  }

  private fun raiseArm() {
    driveController.b().onTrue(
      robot.pivot.setPosition(0.25)
    )
  }

  private fun lowerArm() {
    driveController.a().onTrue(
      robot.pivot.setPosition(0.0)
    )
  }

  private fun raiseElevator() {
    driveController.x().onTrue(
      robot.elevator.setPosition(ElevatorConstants.HIGH_HEIGHT)
    )
  }

  private fun lowerElevator() {
    driveController.y().onTrue(
      robot.elevator.setPosition(ElevatorConstants.STOW_HEIGHT)
    )
  }

  private fun pivotChar() {
    val pivotRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds.of(1.0)),
        Volts.of(4.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      SysIdRoutine.Mechanism(
        { voltage: Measure<Voltage> -> run { robot.pivot.setVoltage(voltage) } },
        null,
        robot.pivot,
        "pivot"
      )
    )

    // Quasistatic Forwards
    driveController.a().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.b().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.x().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.y().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )

  }

  private fun elevatorChar() {
    val elevatorRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Seconds.of(1.0)),
        Volts.of(12.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      SysIdRoutine.Mechanism(
        { voltage: Measure<Voltage> -> run { robot.elevator.setVoltage(voltage.`in`(Volts)) } },
        null,
        robot.elevator,
        "elevator"
      )
    )

    // Quasistatic Forwards
    driveController.a().onTrue(
      elevatorRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.b().onTrue(
      elevatorRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.x().onTrue(
      elevatorRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.y().onTrue(
      elevatorRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  /** Characterization functions */
  private fun characterizationExample() {
    /** Example
     *
     val exampleSubsystemRoutine = SysIdRoutine(
     SysIdRoutine.Config(
     Volts.of(0.5).per(Seconds.of(1.0)),
     Volts.of(3.0),
     Seconds.of(10.0)
     ) { state -> SignalLogger.writeString("state", state.toString()) },
     Mechanism(
     { voltage: Measure<Voltage> ->
     run { robot.shooter.setVoltage(voltage.`in`(Volts)) }
     },
     null,
     robot.shooter,
     "shooter"
     )
     )

     // Quasistatic Forwards
     driveController.povUp().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kForward)
     )

     // Quasistatic Reverse
     driveController.povDown().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
     )

     // Dynamic Forwards
     driveController.povRight().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kForward)
     )

     // Dynamic Reverse
     driveController.povLeft().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kReverse)
     )
     */
  }

  /** Try not to touch, just add things to the robot or nonrobot bindings */
  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
