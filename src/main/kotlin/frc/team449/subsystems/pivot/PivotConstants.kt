package frc.team449.subsystems.pivot

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.util.Color8Bit

object PivotConstants {
  const val MOTOR_ID = 3

  const val UPDATE_FREQUENCY = 4.0
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.CounterClockwise_Positive

  /** Current Homing constants */
  val HOMING_VOLTAGE = Volts.of(2.0)
  val HOMING_TIME_CUTOFF = Seconds.of(4.0)
  val HOMING_CURRENT_CUTOFF = Amps.of(20.0)
  val HOMING_MAX_VEL = RotationsPerSecond.of(0.05)

  val MIN_ANGLE = Degrees.of(0.0)
  val MAX_ANGLE = Degrees.of(90.0)

  /** TODO: Figure out exact values for these, especially once hardstop is created
   * Note: 0 deg must  be defined as the inner 1x1 on the wrist is perfectly vertical
   */
  val TRUE_STOW_ANGLE = Degrees.of(91.233851)
  val STOW_ANGLE = Degrees.of(92.5)
  val INTAKE_ANGLE = Degrees.of(12.618689)

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val STATOR_CURRENT_LIMIT = 150.0

  val TOLERANCE = Degrees.of(1.75)

  const val MOMENT_OF_INERTIA = 0.5253

  const val GEARING_MOTOR_TO_GEARBOX = 20.0 / 1.0
  const val GEARING_GEARBOX_TO_MECHANISM = 4.0 / 1.0
  const val GEARING_MOTOR_TO_MECHANISM = GEARING_MOTOR_TO_GEARBOX * GEARING_GEARBOX_TO_MECHANISM

  const val KS = 0.0
  const val KV = 1.0
  const val KA = 0.0
  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0
  const val KG = 0.0

  const val PIVOT_LENGTH = 1.0
  const val WIDTH = 7.0
  const val TARGET_WIDTH = 4.0
  val REAL_COLOR = Color8Bit(255, 0, 255)
  val TARGET_COLOR = Color8Bit(0, 255, 0)

  const val ACCEL_DAMPING_FACTOR = 0.075

  val CRUISE_VEL = RotationsPerSecond.of(1.0)
  val MAX_ACCEL = RadiansPerSecond.per(Second).of(
    ACCEL_DAMPING_FACTOR * DCMotor.getKrakenX60(1).getTorque(40.0) * GEARING_MOTOR_TO_MECHANISM /
      MOMENT_OF_INERTIA
  )
}
