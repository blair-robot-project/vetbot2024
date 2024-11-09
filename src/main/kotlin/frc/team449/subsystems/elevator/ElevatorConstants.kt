package frc.team449.subsystems.elevator

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object ElevatorConstants {

  const val ANGLE = 75.0 // TODO: someone figure this out
  const val WIDTH = 5.0
  const val TARGET_WIDTH = 8.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)

  const val UPDATE_FREQUENCY = 50.0 // hz
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.CounterClockwise_Positive

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 30.0
  const val STATOR_CURRENT_LIMIT = 80.0
  const val MOTOR_ID = 12

  const val CARRIAGE_MASS_KG = 9.3811974

  const val GEARING_MOTOR_TO_GEAR = 25.0 / 1.0
  const val GEAR_DIAMETER_M = 0.048824388
  const val GEARING_MOTOR_TO_ELEVATOR = GEARING_MOTOR_TO_GEAR * GEAR_DIAMETER_M * PI

  const val MIN_HEIGHT = 0.2054352
  const val STOW_HEIGHT = 0.25 // m
  const val HIGH_HEIGHT = 0.5 // m
  const val MAX_HEIGHT = 0.524383

  const val MM_ACCEL = 4.9 // m/s^2
  const val MM_VEL = 20.0 // m/s

  const val KS = 0.00017165
  const val KV = 1.489
  const val KA = 0.19807
  const val KG = 1.9784
  const val KP = 2.2263
  const val KI = 0.5
  const val KD = 0.0
}
