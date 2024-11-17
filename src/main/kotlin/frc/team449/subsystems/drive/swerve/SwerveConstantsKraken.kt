package frc.team449.subsystems.drive.swerve

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object SwerveConstantsKraken {
  const val EFFICIENCY = 0.95
  const val USE_FOC = false
  val NEUTRAL_MODE = NeutralModeValue.Brake
  const val DUTY_CYCLE_DEADBAND = 0.001
  const val POS_UPDATE_FREQUENCY = 100.0
  const val UPDATE_FREQUENCY = 50.0

  const val SUPPLY_LIMIT = 50.0
  const val SUPPLY_BOOST = 65.0
  const val SUPPLY_BOOST_TIME = 0.25
  const val STATOR_LIMIT = 120.0
  const val CLOSED_LOOP_RAMP = 0.020

  const val TORQUE_CURRENT_LIMIT = 80.0

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 2
  const val DRIVE_MOTOR_FR = 5
  const val DRIVE_MOTOR_BL = 7
  const val DRIVE_MOTOR_BR = 4
  const val TURN_MOTOR_FL = 1
  const val TURN_MOTOR_FR = 6
  const val TURN_MOTOR_BL = 8
  const val TURN_MOTOR_BR = 3

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 6
  const val TURN_ENC_CHAN_FR = 7
  const val TURN_ENC_CHAN_BL = 8
  const val TURN_ENC_CHAN_BR = 9

  /** Offsets for the absolute encoders in rotations. */
  const val TURN_ENC_OFFSET_FL = 0.1348
  const val TURN_ENC_OFFSET_FR = 0.2927 + 0.5
  const val TURN_ENC_OFFSET_BL = 0.4755
  const val TURN_ENC_OFFSET_BR = 0.391 + 0.5

  /** PID gains for turning each module */
  const val TURN_KP = 1.05
  const val TURN_KI = 0.0
  const val TURN_KD = 0.075

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.20285 + 0.02
  const val DRIVE_KV = 2.3887 + 0.2 + 0.0935
  const val DRIVE_KA = 0.43365 + 0.035 + 0.0185

  // TODO: Figure out this value
  const val STEER_KS = 0.05

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.475
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration, Look through SwerveConstantsNEO for turning NEO */
  const val DRIVE_GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
  val DRIVE_UPR = Units.inchesToMeters(4.0) * PI
  const val TURN_UPR = 2 * Math.PI
  val MAX_ATTAINABLE_MK4I_SPEED = Units.feetToMeters(16.35) // (12 - DRIVE_KS) / DRIVE_KV
  const val STEERING_CURRENT_LIM = 40
  const val JOYSTICK_FILTER_ORDER = 2
  const val ROT_FILTER_ORDER = 1.25
  const val SKEW_CONSTANT = 15.5

  /** Wheelbase = wheel-to-wheel distance from front to back of the robot */
  /** Trackwidth = wheel-to-wheel distance from side to side of the robot */
  /** X_SHIFT is a relic from our 2024 robot, where the frame perimeter extends beyond the wheels in the X direction */
  val WHEELBASE = Units.inchesToMeters(16.75) // ex. FL to BL
  val TRACKWIDTH = Units.inchesToMeters(21.75) // ex. BL to BR
  val X_SHIFT = 0.0
}
