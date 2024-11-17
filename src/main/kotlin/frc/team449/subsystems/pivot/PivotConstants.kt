package frc.team449.subsystems.pivot

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.util.Color8Bit

object PivotConstants {
  const val MOTOR_ID = 10

  const val UPDATE_FREQUENCY = 50.0
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.CounterClockwise_Positive

  val MIN_ANGLE = Degrees.of(0.0)
  val MAX_ANGLE = Degrees.of(90.0)

  /** TODO: Figure out exact values for these, especially once hardstop is created
   * Note: 0 deg must  be defined as the inner 1x1 on the wrist is perfectly vertical
   */
  val STOW_ANGLE = Degrees.of(91.5)
  val INTAKE_ANGLE = Degrees.of(5.0)
  val HIGH_ANGLE = Degrees.of(75.0)
  val STACK_ANGLE = Degrees.of(0.0)

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val STATOR_CURRENT_LIMIT = 150.0

  val TOLERANCE = Degrees.of(1.75)

  const val MOMENT_OF_INERTIA = 0.5253

  const val GEARING_MOTOR_TO_GEARBOX = 15.0 / 1.0
  const val GEARING_GEARBOX_TO_MECHANISM = 4.0 / 1.0
  const val GEARING_MOTOR_TO_MECHANISM = GEARING_MOTOR_TO_GEARBOX * GEARING_GEARBOX_TO_MECHANISM

  const val KS = 0.010968
  const val KV = 7.1249
  const val KA = 0.072661
  const val KP = 7.537
  const val KI = 0.0
  const val KD = 0.0
  const val KG = 0.21875

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
