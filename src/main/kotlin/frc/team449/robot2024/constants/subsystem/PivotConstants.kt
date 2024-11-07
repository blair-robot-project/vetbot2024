package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color8Bit

object PivotConstants {

  const val UPDATE_FREQUENCY = 50.0
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.Clockwise_Positive

  val MIN_ANGLE = Units.degreesToRadians(0.0)
  val MAX_ANGLE = Units.degreesToRadians(105.0)

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val STATOR_CURRENT_LIMIT = 150.0
  const val MOTOR_ID = 10

  const val MOMENT_OF_INERTIA = 1.0 // TODO: find actual value

  const val GEARING_MOTOR_TO_GEARBOX = 15.0 / 1.0
  const val GEARING_GEARBOX_TO_MECHANISM = 4.0 / 1.0
  const val GEARING_MOTOR_TO_MECHANISM = GEARING_MOTOR_TO_GEARBOX * GEARING_GEARBOX_TO_MECHANISM

  val MODEL_POS_DEVIATION = Units.degreesToRadians(10.0)
  val MODEL_VEL_DEVIATION = Units.degreesToRadians(20.0)
  val ENCODER_POS_DEVIATION = Units.degreesToRadians(0.175)

  val POS_TOLERANCE = Units.degreesToRadians(1.975)
  val VEL_TOLERANCE = Units.degreesToRadians(13.5)
  const val CONTROL_EFFORT_VOLTS = 12.0

  const val KS = 1.0
  const val KV = 1.0
  const val KA = 1.0
  const val KP = 1.0
  const val KI = 1.0
  const val KD = 1.0
  const val KG = 1.0

  const val PIVOT_LENGTH = 1.0
  const val START_ANGLE = 0.0
  const val WIDTH = 0.25
  val REAL_COLOR = Color8Bit(255, 0, 255)
  val TARGET_COLOR = Color8Bit(0, 255, 0)

  const val CRUISE_VEL = 3.0
  const val MAX_ACCEL = 3.0
}
