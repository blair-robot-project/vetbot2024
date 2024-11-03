package frc.team449.robot2024.constants.subsystem

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object PivotConstants {

  const val UPDATE_FREQUENCY = 50.0
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.Clockwise_Positive

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 40.0
  const val STATOR_CURRENT_LIMIT = 150.0
  const val MOTOR_ID = 0

  const val GEARING_MOTOR_TO_GEARBOX = 15.0 / 1.0
  const val GEARING_GEARBOX_TO_AXLE = 4.0 / 1.0
  const val GEARING = GEARING_MOTOR_TO_GEARBOX * GEARING_GEARBOX_TO_AXLE

  const val RIGHT_KS = 1.0
  const val RIGHT_KV = 1.0
  const val RIGHT_KA = 1.0
  const val RIGHT_KP = 1.0
  const val RIGHT_KI = 1.0
  const val RIGHT_KD = 1.0
}
