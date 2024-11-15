package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.holonomic.HolonomicDrive
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken.Companion.createKrakenModule
import frc.team449.subsystems.drive.swerve.SwerveModuleNEO.Companion.createNEOModule
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.system.AHRS
import kotlin.math.hypot

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param ahrs The gyro that is mounted on the chassis.
 * @param maxLinearSpeed The maximum translation speed of the chassis.
 * @param maxRotSpeed The maximum rotation speed of the chassis.
 * @param cameras The cameras that help estimate the robot's pose.
 * @param field The SmartDashboard [Field2d] widget that shows the robot's pose.
 */
open class SwerveDrive(
  protected val modules: List<SwerveModule>,
  protected val ahrs: AHRS,
  override var maxLinearSpeed: Double,
  override var maxRotSpeed: Double,
  protected val field: Field2d
) : SubsystemBase(), HolonomicDrive {

  /** Vision statistics */

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  protected val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  /** Current estimated vision pose */

  /** Pose estimator that estimates the robot's position as a [Pose2d]. */
  protected val poseEstimator = SwerveDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE,
    VisionConstants.ENCODER_TRUST,
    VisionConstants.MULTI_TAG_TRUST
  )

  init {
    SmartDashboard.putData("Elastic Swerve Drive") { builder: SendableBuilder ->
      builder.setSmartDashboardType("SwerveDrive")
      builder.addDoubleProperty("Front Left Angle", { modules[0].state.angle.radians }, null)
      builder.addDoubleProperty("Front Left Velocity", { modules[0].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Front Right Angle", { modules[1].state.angle.radians }, null)
      builder.addDoubleProperty("Front Right Velocity", { modules[1].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Left Angle", { modules[2].state.angle.radians }, null)
      builder.addDoubleProperty("Back Left Velocity", { modules[2].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Right Angle", { modules[3].state.angle.radians }, null)
      builder.addDoubleProperty("Back Right Velocity", { modules[3].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Robot Angle", { heading.radians }, null)
    }
  }

  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  protected var speedMagnitude: Double = 0.0

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds

    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
//    normalizeDrive(desiredModuleStates, desiredSpeeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()
  }

  fun setVoltage(volts: Double) {
    modules.forEach {
      it.setVoltage(volts)
    }
  }

  fun getModuleVel(): Double {
    var totalVel = 0.0
    modules.forEach { totalVel += it.state.speedMetersPerSecond }
    return totalVel / modules.size
  }

  /** The measured pitch of the robot from the gyro sensor. */
  val pitch: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.pitch.radians))

  /** The measured roll of the robot from the gyro sensor. */
  val roll: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.roll.radians))

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        getPositions(),
        value
      )
    }

  override fun periodic() {
    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      arrayOf(
        modules[0].state,
        modules[1].state,
        modules[2].state,
        modules[3].state
      )
    )

    speedMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)

    // Update the robot's pose using the gyro heading and the SwerveModulePositions of each module.
    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    // Sets the robot's pose and individual module rotations on the SmartDashboard [Field 2d] widget.
    setRobotPose()
  }

  /** Stops the robot's drive. */
  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  /** @return An array of [SwerveModulePosition] for each module, containing distance and angle. */
  protected fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /** @return An array of [SwerveModuleState] for each module, containing speed and angle. */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  protected fun setRobotPose() {
    this.field.robotPose = this.pose

    this.field.getObject("FL").pose = this.pose.plus(
      Transform2d(
        Translation2d(
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
        ),
        this.getPositions()[0].angle
      )
    )

    this.field.getObject("FR").pose = this.pose.plus(
      Transform2d(
        Translation2d(
          SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
        ),
        this.getPositions()[1].angle
      )
    )

    this.field.getObject("BL").pose = this.pose.plus(
      Transform2d(
        Translation2d(
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          SwerveConstants.TRACKWIDTH / 2
        ),
        this.getPositions()[2].angle
      )
    )

    this.field.getObject("BR").pose = this.pose.plus(
      Transform2d(
        Translation2d(
          -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
          -SwerveConstants.TRACKWIDTH / 2
        ),
        this.getPositions()[0].angle
      )
    )
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Poses and ChassisSpeeds")
    builder.addDoubleArrayProperty("1.1 Estimated Pose", { doubleArrayOf(pose.x, pose.y, pose.rotation.radians) }, null)
    builder.addDoubleArrayProperty("1.2 Current Chassis Vel", { doubleArrayOf(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleArrayProperty("1.3 Desired Chassis Vel", { doubleArrayOf(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleProperty("1.4 Magnitude of Vel", { speedMagnitude }, null)

    builder.publishConstString("2.0", "Vision Stats")

    builder.publishConstString("3.0", "Driving & Steering (Std Order FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "3.1 Current States",
      {
        doubleArrayOf(
          modules[0].state.angle.radians,
          modules[0].state.speedMetersPerSecond,
          modules[1].state.angle.radians,
          modules[1].state.speedMetersPerSecond,
          modules[2].state.angle.radians,
          modules[2].state.speedMetersPerSecond,
          modules[3].state.angle.radians,
          modules[3].state.speedMetersPerSecond,
        )
      },
      null
    )
    builder.addDoubleArrayProperty(
      "3.2 Desired States",
      {
        doubleArrayOf(
          modules[0].desiredState.angle.radians,
          modules[0].desiredState.speedMetersPerSecond,
          modules[1].desiredState.angle.radians,
          modules[1].desiredState.speedMetersPerSecond,
          modules[2].desiredState.angle.radians,
          modules[2].desiredState.speedMetersPerSecond,
          modules[3].desiredState.angle.radians,
          modules[3].desiredState.speedMetersPerSecond,
        )
      },
      null
    )

    builder.addDoubleArrayProperty(
      "3.3 Steering Rotation",
      {
        doubleArrayOf(
          modules[0].state.angle.rotations,
          modules[1].state.angle.rotations,
          modules[2].state.angle.rotations,
          modules[3].state.angle.rotations,
        )
      },
      null
    )

    builder.publishConstString("5.0", "AHRS Values")
    builder.addDoubleProperty("5.1 Heading Degrees", { ahrs.heading.degrees }, null)
    builder.addDoubleProperty("5.2 Pitch Degrees", { ahrs.pitch.degrees }, null)
    builder.addDoubleProperty("5.3 Roll Degrees", { ahrs.roll.degrees }, null)
    builder.addDoubleProperty("5.4 Angular X Vel", { ahrs.angularXVel() }, null)
    builder.addBooleanProperty("5.5 Navx Connected", { ahrs.connected() }, null)
    builder.addBooleanProperty("5.6 Navx Calibrated", { ahrs.calibrated() }, null)
  }

  companion object {
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerveKraken(ahrs: AHRS, field: Field2d): SwerveDrive {
      val modules = listOf(
        createKrakenModule(
          "FLModule",
          SwerveConstants.DRIVE_MOTOR_FL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FL,
          SwerveConstants.TURN_ENC_OFFSET_FL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "FRModule",
          SwerveConstants.DRIVE_MOTOR_FR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FR,
          SwerveConstants.TURN_ENC_OFFSET_FR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BL,
          SwerveConstants.TURN_ENC_OFFSET_BL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BR,
          SwerveConstants.TURN_ENC_OFFSET_BR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field,
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field,
        )
      }
    }

    fun createSwerveNEO(ahrs: AHRS, field: Field2d): SwerveDrive {
      val modules = listOf(
        createNEOModule(
          "FLModule",
          SwerveConstants.DRIVE_MOTOR_FL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FL,
          SwerveConstants.TURN_ENC_OFFSET_FL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "FRModule",
          SwerveConstants.DRIVE_MOTOR_FR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FR,
          SwerveConstants.TURN_ENC_OFFSET_FR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BL,
          SwerveConstants.TURN_ENC_OFFSET_BL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BR,
          SwerveConstants.TURN_ENC_OFFSET_BR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field
        )
      }
    }
  }
}
