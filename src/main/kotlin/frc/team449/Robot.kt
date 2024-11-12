package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.elevator.Elevator.Companion.createElevator
import frc.team449.subsystems.intake.Intake.Companion.createIntake
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.pivot.Pivot.Companion.createPivot
import frc.team449.system.AHRS
import monologue.Annotations.Log
import monologue.Logged

class Robot : RobotBase(), Logged {

  val driveController = CommandXboxController(0)

  val mechController = CommandXboxController(1)

  val ahrs = AHRS()

  // Instantiate/declare PDP and other stuff here

  @Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  val intake = createIntake()

  @Log.NT
  val pivot = createPivot()

  @Log.NT
  val elevator = createElevator()

  @Log.NT
  override val drive = SwerveDrive.createSwerveNEO(ahrs, field)

  @Log.NT
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController.hid)

  val light = createLight()

  /** Example for using infrared sensors
   * @Log.NT
   * val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
   */
}
