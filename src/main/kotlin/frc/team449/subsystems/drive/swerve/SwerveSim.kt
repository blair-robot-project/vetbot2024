package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.subsystems.RobotConstants
import frc.team449.system.AHRS
import kotlin.math.hypot
import kotlin.random.Random

class SwerveSim(
  modules: List<SwerveModule>,
  ahrs: AHRS,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  field: Field2d
) : SwerveDrive(modules, ahrs, maxLinearSpeed, maxRotSpeed, field) {

  private var lastTime = getFPGATimestamp()
  var odoPose = Pose2d()
  var currHeading = Rotation2d()

  private val odometry = SwerveDriveOdometry(
    kinematics,
    currHeading,
    getPositions(),
    RobotConstants.INITIAL_POSE
  )

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        currHeading,
        getPositions(),
        value
      )

      odometry.resetPosition(
        currHeading,
        getPositions(),
        value
      )
    }

  fun resetPos() {
    val newPose = Pose2d(
      poseEstimator.estimatedPosition.x + Random.nextDouble(-1.0, 1.0),
      poseEstimator.estimatedPosition.y + Random.nextDouble(-1.0, 1.0),
      poseEstimator.estimatedPosition.rotation
    )

    poseEstimator.resetPosition(
      currHeading,
      getPositions(),
      newPose
    )
  }

  override fun periodic() {
    val currTime = getFPGATimestamp()

    currHeading = currHeading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    set(super.desiredSpeeds)

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
      currHeading,
      getPositions()
    )

    // Sets the robot's pose and individual module rotations on the SmartDashboard [Field2d] widget.
    setRobotPose()

    odoPose = odometry.update(
      currHeading,
      getPositions()
    )

    field.getObject("odo").pose = odoPose
  }
}
