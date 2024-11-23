package frc.team449.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import frc.team449.subsystems.drive.swerve.SwerveConstantsNEO
import kotlin.math.PI

object RobotConstants {

  /** Other CAN ID */
  const val PDH_CAN = 1

  /** Controller Configurations */
  const val ROT_RATE_LIMIT = 17.27 * PI
  const val NEG_ROT_RATE_LIM = -27.5 * PI
  const val DRIVE_RADIUS_DEADBAND = .125
  const val ROTATION_DEADBAND = .125
  val SNAP_TO_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(1.0)

  /** In kilograms, include bumpers and battery and all */
  const val ROBOT_WEIGHT = 55.0 // TODO: find

  /** Drive configuration */
  val MAX_LINEAR_SPEED = SwerveConstantsNEO.MAX_ATTAINABLE_MK4I_SPEED // m/s
  const val MAX_ROT_SPEED = 5.804 * PI / 4 // rad/s

  const val USE_ACCEL_LIMIT = true

  val MAX_ACCEL = 4 *
    DCMotor.getNEO(1)
      .getTorque(75.0) /
    ((SwerveConstantsNEO.DRIVE_UPR / (2 * PI)) * ROBOT_WEIGHT * SwerveConstantsNEO.DRIVE_GEARING) // m/s/s

  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d())

  init {
    println("Drive Max Accel: $MAX_ACCEL")
  }

  const val LOOP_TIME = 0.020 //

  /** PID controller for snap to angle turning */
  val SNAP_KP = 5.85
  val SNAP_KI = 0.0
  val SNAP_KD = 0.0

  const val ALIGN_ROT_SPEED = 7 * PI / 2

  // Robot dimensions (INCLUDING BUMPERS)
  val ROBOT_WIDTH = Units.inchesToMeters(27.25 + 3.25 * 2)
  val ROBOT_LENGTH = Units.inchesToMeters(27.5 + 3.25 * 2)
}
