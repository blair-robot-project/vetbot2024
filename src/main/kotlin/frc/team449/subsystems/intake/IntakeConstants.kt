package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.Amps

object IntakeConstants {
  const val LEADER_ID = 13
  const val FOLLOWER_ID = 14

  const val LEADER_INVERTED = true
  const val FOLLOWER_INVERTED_FROM_LEADER = false

  const val BRAKE_MODE = false
  val CURRENT_LIMIT = Amps.of(40.0)

  const val INTAKE_VOLTAGE = 6.0
  const val HOLD_VOLTAGE = 2.0
  const val OUTTAKE_VOLTAGE = -3.0
}
