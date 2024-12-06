package frc.team449.subsystems.elevator

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object ElevatorConstants {

  const val MOTOR_ID = 11

  const val ANGLE = 75.0 // DEGREES
  const val WIDTH = 7.0
  const val TARGET_WIDTH = 4.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)
  const val MAX_HEIGHT = 0.524383
  const val MIN_HEIGHT = 0.2054352

  /** Current Homing constants */
  val HOMING_VOLTAGE = Units.Volts.of(-2.0)
  val HOMING_TIME_CUTOFF = Units.Seconds.of(4.0)
  val HOMING_CURRENT_CUTOFF = Units.Amps.of(20.0)
  val HOMING_MAX_VEL = Units.MetersPerSecond.of(0.05)

  const val UPDATE_FREQUENCY = 50.0 // hz
  const val DUTY_CYCLE_DEADBAND = 0.001
  val NEUTRAL_MODE = NeutralModeValue.Brake
  val ORIENTATION = InvertedValue.CounterClockwise_Positive

  const val BURST_TIME_LIMIT = 0.25
  const val BURST_CURRENT_LIMIT = 60.0
  const val SUPPLY_CURRENT_LIMIT = 30.0
  const val STATOR_CURRENT_LIMIT = 80.0

  const val TOLERANCE = 0.025

  const val CARRIAGE_MASS_KG = 9.3811974

  const val GEARING_MOTOR_TO_GEAR = 20.0 / 1.0 // 03-12-2024
  const val GEAR_DIAMETER_M = 0.048824388
  const val GEARING_MOTOR_TO_ELEVATOR = GEARING_MOTOR_TO_GEAR / (GEAR_DIAMETER_M * PI)

  const val STOW_HEIGHT = 0.0 // m
  const val HIGH_HEIGHT = 0.5 // m

  const val ACCEL_DAMPING = 0.05

  const val MM_JERK = 0.0 // m/s^3
  val MM_ACCEL = ACCEL_DAMPING * (
    DCMotor.getKrakenX60(1).getTorque(40.0) *
      GEARING_MOTOR_TO_GEAR / (GEAR_DIAMETER_M / 2)
    ) / CARRIAGE_MASS_KG

  init {
    print("Elevator accel: ")
    println(MM_ACCEL)
  }

  const val MM_VEL = (5800 / 60) / GEARING_MOTOR_TO_ELEVATOR

  // TODO replace with real sysid values
  const val KS = 0.0
  const val KV = 12.0 / ((5800 / 60) / GEARING_MOTOR_TO_ELEVATOR)
  const val KA = 0.0
  const val KG = 0.0
  const val KP = 6.0
  const val KI = 0.0
  const val KD = 0.0
}
