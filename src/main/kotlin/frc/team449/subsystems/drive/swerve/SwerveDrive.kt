package frc.team449.subsystems.drive.swerve

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
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
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.holonomic.HolonomicDrive
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.subsystems.vision.VisionSubsystem
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt

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
  protected val cameras: List<VisionSubsystem> = mutableListOf(),
  protected val field: Field2d,
  private val trackwidth: Double,
  private val wheelbase: Double,
  private val xShift: Double,
  private val maxAttainableModuleSpeed: Double
) : SubsystemBase(), HolonomicDrive {

  /** Vision statistics */
  protected val numTargets = DoubleArray(cameras.size)
  protected val tagDistance = DoubleArray(cameras.size)
  protected val avgAmbiguity = DoubleArray(cameras.size)
  protected val heightError = DoubleArray(cameras.size)
  protected val usedVision = BooleanArray(cameras.size)
  protected val usedVisionSights = LongArray(cameras.size)
  protected val rejectedVisionSights = LongArray(cameras.size)
  var visionRunning = false

  var enableVisionFusion = true

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  protected val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  /** Current estimated vision pose */
  var visionPose = DoubleArray(cameras.size * 3)

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
      maxAttainableModuleSpeed
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
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    speedMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)

    // Update the robot's pose using the gyro heading and the SwerveModulePositions of each module.
    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    val visionPoseCopy = visionPose.clone()

//    if (cameras.isNotEmpty()) localize()

//    visionRunning = visionPose[0] != visionPoseCopy[0] ||
//      visionPose[1] != visionPoseCopy[1] ||
//      visionPose[2] != visionPoseCopy[2]

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
        Translation2d(wheelbase / 2 - xShift, trackwidth / 2),
        this.getPositions()[0].angle
      )
    )

    this.field.getObject("FR").pose = this.pose.plus(
      Transform2d(
        Translation2d(wheelbase / 2 - xShift, -trackwidth / 2),
        this.getPositions()[1].angle
      )
    )

    this.field.getObject("BL").pose = this.pose.plus(
      Transform2d(
        Translation2d(-wheelbase / 2 - xShift, trackwidth / 2),
        this.getPositions()[2].angle
      )
    )

    this.field.getObject("BR").pose = this.pose.plus(
      Transform2d(
        Translation2d(-wheelbase / 2 - xShift, -trackwidth / 2),
        this.getPositions()[0].angle
      )
    )
  }

  protected open fun localize() = try {
    for ((index, camera) in cameras.withIndex()) {
      val result = camera.estimatedPose(Pose2d(pose.x, pose.y, ahrs.heading))
      if (result.isPresent) {
        val presentResult = result.get()
        numTargets[index] = presentResult.targetsUsed.size.toDouble()
        tagDistance[index] = 0.0
        avgAmbiguity[index] = 0.0
        heightError[index] = abs(presentResult.estimatedPose.z)

        for (tag in presentResult.targetsUsed) {
          val tagPose = camera.estimator.fieldTags.getTagPose(tag.fiducialId)
          if (tagPose.isPresent) {
            val estimatedToTag = presentResult.estimatedPose.minus(tagPose.get())
            tagDistance[index] += sqrt(estimatedToTag.x.pow(2) + estimatedToTag.y.pow(2)) / numTargets[index]
            avgAmbiguity[index] = tag.poseAmbiguity / numTargets[index]
          } else {
            tagDistance[index] = Double.MAX_VALUE
            avgAmbiguity[index] = Double.MAX_VALUE
            break
          }
        }

        val estVisionPose = presentResult.estimatedPose.toPose2d()

        visionPose[0 + 3 * index] = estVisionPose.x
        visionPose[1 + 3 * index] = estVisionPose.y
        visionPose[2 + 3 * index] = estVisionPose.rotation.radians

        if (presentResult.timestampSeconds > 0 &&
          avgAmbiguity[index] <= VisionConstants.MAX_AMBIGUITY &&
          numTargets[index] < 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_SINGLE_TAG ||
          numTargets[index] >= 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_MULTI_TAG + (numTargets[index] - 2) * VisionConstants.NUM_TAG_FACTOR &&
          heightError[index] < VisionConstants.MAX_HEIGHT_ERR_METERS
        ) {
          if (enableVisionFusion) {
            poseEstimator.addVisionMeasurement(
              estVisionPose,
              presentResult.timestampSeconds,
              camera.getEstimationStdDevs(numTargets[index].toInt(), tagDistance[index])
            )
          }
          usedVision[index] = true
          usedVisionSights[index] += 1.toLong()
        } else {
          usedVision[index] = false
          rejectedVisionSights[index] += 1.toLong()
        }
      }
    }
  } catch (e: Error) {
    DriverStation.reportError(
      "!!!!!!!!! VISION ERROR !!!!!!!",
      e.stackTrace
    )
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Poses and ChassisSpeeds")
    builder.addDoubleArrayProperty("1.1 Estimated Pose", { doubleArrayOf(pose.x, pose.y, pose.rotation.radians) }, null)
    builder.addDoubleArrayProperty("1.2 Current Chassis Vel", { doubleArrayOf(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleArrayProperty("1.3 Desired Chassis Vel", { doubleArrayOf(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleProperty("1.4 Magnitude of Vel", { speedMagnitude }, null)

    builder.publishConstString("2.0", "Vision Stats")
    builder.addBooleanArrayProperty("2.1 Used Last Vision Estimate?", { usedVision }, null)
    builder.addDoubleArrayProperty("2.2 Number of Targets", { numTargets }, null)
    builder.addDoubleArrayProperty("2.3 Avg Tag Distance", { tagDistance }, null)
    builder.addDoubleArrayProperty("2.4 Average Ambiguity", { avgAmbiguity }, null)
    builder.addDoubleArrayProperty("2.5 Cam Height Error", { heightError }, null)
    builder.addIntegerArrayProperty("2.6 Total Used Vision Sights", { usedVisionSights }, null)
    builder.addIntegerArrayProperty("2.7 Total Rejected Vision Sights", { rejectedVisionSights }, null)
    for ((index, _) in cameras.withIndex()) {
      builder.addDoubleArrayProperty("2.8${1 + index} Vision Pose Cam $index", { visionPose.slice(IntRange(0 + 3 * index, 2 + 3 * index)).toDoubleArray() }, null)
    }
    builder.addBooleanProperty("2.9 Enabled Vision Fusion", { enableVisionFusion }, null)
    builder.addBooleanProperty("2.91 New Vision Measurement", { visionRunning }, null)

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
    /** Create a [SwerveDrive] using [SwerveConstantsNEO]. */
    fun createSwerveKraken(ahrs: AHRS, field: Field2d): SwerveDrive {
      val turnMotorController = { PIDController(SwerveConstantsKraken.TURN_KP, SwerveConstantsKraken.TURN_KI, SwerveConstantsKraken.TURN_KD) }
      val modules = listOf(
        SwerveModuleKraken.create(
          "FLModule",
          makeKrakenDrivingMotor(
            SwerveConstantsKraken.DRIVE_MOTOR_FL,
            inverted = InvertedValue.CounterClockwise_Positive
          ),
          makeNEOTurningMotorKrakenConstants(
            "FL",
            SwerveConstantsKraken.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsKraken.TURN_ENC_CHAN_FL,
            SwerveConstantsKraken.TURN_ENC_OFFSET_FL
          ),
          turnMotorController(),
          Translation2d(SwerveConstantsKraken.WHEELBASE / 2 - SwerveConstantsKraken.X_SHIFT, SwerveConstantsKraken.TRACKWIDTH / 2)
        ),
        SwerveModuleKraken.create(
          "FRModule",
          makeKrakenDrivingMotor(
            SwerveConstantsKraken.DRIVE_MOTOR_FR,
            inverted = InvertedValue.CounterClockwise_Positive
          ),
          makeNEOTurningMotorKrakenConstants(
            "FR",
            SwerveConstantsKraken.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsKraken.TURN_ENC_CHAN_FR,
            SwerveConstantsKraken.TURN_ENC_OFFSET_FR
          ),
          turnMotorController(),
          Translation2d(SwerveConstantsKraken.WHEELBASE / 2 - SwerveConstantsKraken.X_SHIFT, -SwerveConstantsKraken.TRACKWIDTH / 2)
        ),
        SwerveModuleKraken.create(
          "BLModule",
          makeKrakenDrivingMotor(
            SwerveConstantsKraken.DRIVE_MOTOR_BL,
            inverted = InvertedValue.CounterClockwise_Positive
          ),
          makeNEOTurningMotorKrakenConstants(
            "BL",
            SwerveConstantsKraken.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsKraken.TURN_ENC_CHAN_BL,
            SwerveConstantsKraken.TURN_ENC_OFFSET_BL
          ),
          turnMotorController(),
          Translation2d(-SwerveConstantsKraken.WHEELBASE / 2 - SwerveConstantsKraken.X_SHIFT, SwerveConstantsKraken.TRACKWIDTH / 2)
        ),
        SwerveModuleKraken.create(
          "BRModule",
          makeKrakenDrivingMotor(
            SwerveConstantsKraken.DRIVE_MOTOR_BR,
            inverted = InvertedValue.CounterClockwise_Positive
          ),
          makeNEOTurningMotorKrakenConstants(
            "BR",
            SwerveConstantsKraken.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsKraken.TURN_ENC_CHAN_BR,
            SwerveConstantsKraken.TURN_ENC_OFFSET_BR
          ),
          turnMotorController(),
          Translation2d(-SwerveConstantsKraken.WHEELBASE / 2 - SwerveConstantsKraken.X_SHIFT, -SwerveConstantsKraken.TRACKWIDTH / 2)
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field,
          SwerveConstantsKraken.TRACKWIDTH,
          SwerveConstantsKraken.WHEELBASE,
          SwerveConstantsKraken.X_SHIFT,
          SwerveConstantsKraken.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field,
          SwerveConstantsKraken.TRACKWIDTH,
          SwerveConstantsKraken.WHEELBASE,
          SwerveConstantsKraken.X_SHIFT,
          SwerveConstantsKraken.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }

    fun createSwerveNEO(ahrs: AHRS, field: Field2d): SwerveDrive {
      val driveMotorController = { PIDController(SwerveConstantsNEO.DRIVE_KP, SwerveConstantsNEO.DRIVE_KI, SwerveConstantsNEO.DRIVE_KD) }
      val turnMotorController = { PIDController(SwerveConstantsNEO.TURN_KP, SwerveConstantsNEO.TURN_KI, SwerveConstantsNEO.TURN_KD) }
      val driveFeedforward = SimpleMotorFeedforward(SwerveConstantsNEO.DRIVE_KS, SwerveConstantsNEO.DRIVE_KV, SwerveConstantsNEO.DRIVE_KA)
      val modules = listOf(
        SwerveModuleNEO.create(
          "FLModule",
          makeNEODrivingMotor(
            "FL",
            SwerveConstantsNEO.DRIVE_MOTOR_FL,
            inverted = false
          ),
          makeNEOTurningMotor(
            "FL",
            SwerveConstantsNEO.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsNEO.TURN_ENC_CHAN_FL,
            SwerveConstantsNEO.TURN_ENC_OFFSET_FL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstantsNEO.WHEELBASE / 2 - SwerveConstantsNEO.X_SHIFT, SwerveConstantsNEO.TRACKWIDTH / 2)
        ),
        SwerveModuleNEO.create(
          "FRModule",
          makeNEODrivingMotor(
            "FR",
            SwerveConstantsNEO.DRIVE_MOTOR_FR,
            inverted = false
          ),
          makeNEOTurningMotor(
            "FR",
            SwerveConstantsNEO.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsNEO.TURN_ENC_CHAN_FR,
            SwerveConstantsNEO.TURN_ENC_OFFSET_FR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstantsNEO.WHEELBASE / 2 - SwerveConstantsNEO.X_SHIFT, -SwerveConstantsNEO.TRACKWIDTH / 2)
        ),
        SwerveModuleNEO.create(
          "BLModule",
          makeNEODrivingMotor(
            "BL",
            SwerveConstantsNEO.DRIVE_MOTOR_BL,
            inverted = false
          ),
          makeNEOTurningMotor(
            "BL",
            SwerveConstantsNEO.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsNEO.TURN_ENC_CHAN_BL,
            SwerveConstantsNEO.TURN_ENC_OFFSET_BL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstantsNEO.WHEELBASE / 2 - SwerveConstantsNEO.X_SHIFT, SwerveConstantsNEO.TRACKWIDTH / 2)
        ),
        SwerveModuleNEO.create(
          "BRModule",
          makeNEODrivingMotor(
            "BR",
            SwerveConstantsNEO.DRIVE_MOTOR_BR,
            inverted = false
          ),
          makeNEOTurningMotor(
            "BR",
            SwerveConstantsNEO.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstantsNEO.TURN_ENC_CHAN_BR,
            SwerveConstantsNEO.TURN_ENC_OFFSET_BR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstantsNEO.WHEELBASE / 2 - SwerveConstantsNEO.X_SHIFT, -SwerveConstantsNEO.TRACKWIDTH / 2)
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field,
          SwerveConstantsNEO.TRACKWIDTH,
          SwerveConstantsNEO.WHEELBASE,
          SwerveConstantsNEO.X_SHIFT,
          SwerveConstantsNEO.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          VisionConstants.ESTIMATORS,
          field,
          SwerveConstantsNEO.TRACKWIDTH,
          SwerveConstantsNEO.WHEELBASE,
          SwerveConstantsNEO.X_SHIFT,
          SwerveConstantsNEO.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }

    /** Helper to make turning motors for swerve. */
    private fun makeKrakenDrivingMotor(
      motorId: Int,
      inverted: InvertedValue
    ): TalonFX {
      val motor = TalonFX(motorId)

      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = inverted
      config.MotorOutput.NeutralMode = SwerveConstantsKraken.NEUTRAL_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = SwerveConstantsKraken.DUTY_CYCLE_DEADBAND

      config.Feedback.SensorToMechanismRatio = 1 / (SwerveConstantsKraken.DRIVE_GEARING * SwerveConstantsKraken.DRIVE_UPR)

      config.Slot0.kP = SwerveConstantsKraken.DRIVE_KP
      config.Slot0.kI = SwerveConstantsKraken.DRIVE_KI
      config.Slot0.kD = SwerveConstantsKraken.DRIVE_KD
      config.Slot0.kS = SwerveConstantsKraken.DRIVE_KS
      config.Slot0.kV = SwerveConstantsKraken.DRIVE_KV
      config.Slot0.kA = SwerveConstantsKraken.DRIVE_KA

      config.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstantsKraken.TORQUE_CURRENT_LIMIT
      config.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstantsKraken.TORQUE_CURRENT_LIMIT
      config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstantsKraken.CLOSED_LOOP_RAMP

      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = SwerveConstantsKraken.STATOR_LIMIT
      config.CurrentLimits.SupplyCurrentLimit = SwerveConstantsKraken.SUPPLY_LIMIT
      config.CurrentLimits.SupplyCurrentThreshold = SwerveConstantsKraken.SUPPLY_BOOST
      config.CurrentLimits.SupplyTimeThreshold = SwerveConstantsKraken.SUPPLY_BOOST_TIME
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstantsKraken.CLOSED_LOOP_RAMP
      config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstantsKraken.CLOSED_LOOP_RAMP

      var status: StatusCode = StatusCode.StatusCodeNotInitialized
      for (i in 0..4) {
        status = motor.configurator.apply(config)
        if (status.isOK) break
      }
      if (!status.isOK) {
        println("Could not apply configs, error code: $status")
      }

      motor.position.setUpdateFrequency(SwerveConstantsKraken.UPDATE_FREQUENCY)
      motor.velocity.setUpdateFrequency(SwerveConstantsKraken.UPDATE_FREQ_2)
      motor.motorVoltage.setUpdateFrequency(SwerveConstantsKraken.UPDATE_FREQ_2)
      motor.optimizeBusUtilization()

      return motor
    }

    /** Helper to make turning motors for swerve. */
    private fun makeNEODrivingMotor(
      name: String,
      motorId: Int,
      inverted: Boolean
    ) =
      createSparkMax(
        name = name + "Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator =
        NEOEncoder.creator(
          SwerveConstantsNEO.DRIVE_UPR,
          SwerveConstantsNEO.DRIVE_GEARING
        ),
        currentLimit = SwerveConstantsNEO.DRIVE_CURRENT_LIM
      )

    /** Helper to make turning motors for swerve. */
    private fun makeNEOTurningMotor(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstantsNEO.TURN_UPR,
          sensorPhase
        ),
        currentLimit = SwerveConstantsNEO.STEERING_CURRENT_LIM
      )

    /** Helper to make turning motors for swerve. */
    private fun makeNEOTurningMotorKrakenConstants(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstantsKraken.TURN_UPR,
          sensorPhase
        ),
        currentLimit = SwerveConstantsKraken.STEERING_CURRENT_LIM
      )
  }
}
