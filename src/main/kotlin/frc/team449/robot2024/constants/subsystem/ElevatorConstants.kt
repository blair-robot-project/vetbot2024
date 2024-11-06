package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object ElevatorConstants {

  const val ANGLE = 30.0 // TODO: someone figure this out
  const val MIN_LENGTH = 0.1
  const val WIDTH = 10.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)

  const val UPDATE_FREQUENCY = 50.0 // hz
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.Clockwise_Positive

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 30.0
  const val STATOR_CURRENT_LIMIT = 80.0
  const val MOTOR_ID = 12

  const val GEARING_MOTOR_TO_GEAR = 25.0 / 1.0
  const val GEAR_DIAMETER_IN = 1.92222
  const val MOTOR_ROTS_TO_ELEVATOR_IN = GEARING_MOTOR_TO_GEAR * 1 / (GEAR_DIAMETER_IN * PI)
  const val GEARING = MOTOR_ROTS_TO_ELEVATOR_IN

  const val STOW_HEIGHT = 1.0 // m
  const val HIGH_HEIGHT = 2.0 // m

  const val MM_ACCEL = 4.9 // m/s^2
  const val MM_VEL = 20.0 // m/s

  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0
  const val KG = 0.0
  const val KP = 0.0
  const val KI = 0.0
  const val KD = 0.0
}
