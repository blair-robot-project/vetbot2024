package frc.team449.subsystems.intake

import com.revrobotics.spark.SparkMax
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createSparkMax

class Intake(
  private val leader: SparkMax,
  private val follower: SparkMax
) : SubsystemBase() {

  fun intake(): Command {
    return this.runOnce {
      leader.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun hold(): Command {
    return this.runOnce {
      leader.setVoltage(IntakeConstants.HOLD_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      leader.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE)
    }
  }

  // HEIMOV JR CODE DO NOT REMOVE
  fun stop(): Command {
    return this.runOnce {
      leader.stopMotor()
    }
  }

  fun setVoltage(volts: Double): Command {
    return this.runOnce {
      leader.setVoltage(volts)
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltage")
    builder.addDoubleProperty("1.1 Voltage", { leader.get() }, null)
  }

  companion object {
    fun createIntake(): Intake {
      val leader = createSparkMax(
        IntakeConstants.LEADER_ID,
        IntakeConstants.LEADER_INVERTED,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )
      val follower = createFollowerSpark(
        IntakeConstants.FOLLOWER_ID,
        leader,
        IntakeConstants.FOLLOWER_INVERTED_FROM_LEADER
      )

      return Intake(
        leader,
        follower
      )
    }
  }
}
