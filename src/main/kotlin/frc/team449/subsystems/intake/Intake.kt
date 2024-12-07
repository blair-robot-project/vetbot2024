package frc.team449.subsystems.intake

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedNEO
import frc.team449.system.motor.createSparkMax

class Intake(
  private val motor: WrappedNEO
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      motor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setVoltage(IntakeConstants.HOLD_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.run {
      motor.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE)
    }
  }

  // HEIMOV JR CODE DO NOT REMOVE
  fun stop(): Command {
    return this.runOnce {
      motor.stopMotor()
    }
  }

  fun setVoltage(volts: Double): Command {
    return this.runOnce {
      motor.setVoltage(volts)
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltage")
    builder.addDoubleProperty("1.1 Voltage", { motor.get() }, null)
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        id = IntakeConstants.FRONT_ID,
        encCreator = NEOEncoder.creator(1.0, 1.0),
        enableBrakeMode = IntakeConstants.BRAKE_MODE,
        inverted = IntakeConstants.FRONT_INVERTED,
        currentLimit = IntakeConstants.CURRENT_LIMIT,
        slaveSparks = mapOf(
          Pair(IntakeConstants.BACK_ID, IntakeConstants.BACK_INVERTED_FROM_FRONT)
        )
      )

      return Intake(
        motor
      )
    }
  }
}
