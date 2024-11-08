package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit

object PivotConstants {

  const val UPDATE_FREQUENCY = 50.0
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.CounterClockwise_Positive

  val MIN_ANGLE = Units.degreesToRadians(0.0)
  val MAX_ANGLE = Units.degreesToRadians(105.0)

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val STATOR_CURRENT_LIMIT = 150.0
  const val MOTOR_ID = 10

  const val MOMENT_OF_INERTIA = 0.5253

  const val GEARING_MOTOR_TO_GEARBOX = 15.0 / 1.0
  const val GEARING_GEARBOX_TO_MECHANISM = 4.0 / 1.0
  const val GEARING_MOTOR_TO_MECHANISM = GEARING_MOTOR_TO_GEARBOX * GEARING_GEARBOX_TO_MECHANISM

  const val KS = 0.010968
  const val KV = 7.1249
  const val KA = 0.072661
  const val KP = 7.537
  const val KI = 0.5
  const val KD = 0.0
  const val KG = 0.21875

  const val PIVOT_LENGTH = 1.0
  const val START_ANGLE = 0.0
  const val WIDTH = 5.0
  const val TARGET_WIDTH = 8.0
  val REAL_COLOR = Color8Bit(255, 0, 255)
  val TARGET_COLOR = Color8Bit(0, 255, 0)

  const val CRUISE_VEL = 10.0
  const val MAX_ACCEL = 5.0
}
