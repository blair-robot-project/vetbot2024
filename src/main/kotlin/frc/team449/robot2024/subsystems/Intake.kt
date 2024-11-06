package frc.team449.robot2024.subsystems

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.IntakeConstants

class Intake(
  private val frontMotor: CANSparkMax,
  private val backMotor: CANSparkMax
) : SubsystemBase() {
  fun intake(): Command {
    return this.runOnce {
      frontMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      frontMotor.setVoltage(IntakeConstants.OUTTAKE_VOLTAGE)
    }
  }

  fun setVoltage(volts: Double): Command {
    return this.runOnce {
      frontMotor.setVoltage(volts)
    }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Voltages")
    builder.addDoubleProperty("1.1 Front Voltage", { frontMotor.get() }, null)
    builder.addDoubleProperty("1.2 Back Voltage", { backMotor.get() }, null)
  }

  companion object {
    fun createIntake(): Intake {
      val frontMotor = CANSparkMax(IntakeConstants.FRONT_ID, CANSparkLowLevel.MotorType.kBrushless)
      val backMotor = CANSparkMax(IntakeConstants.BACK_ID, CANSparkLowLevel.MotorType.kBrushless)
      frontMotor.inverted = IntakeConstants.FRONT_INVERTED
      backMotor.follow(frontMotor, IntakeConstants.BACK_INVERTED_FROM_FRONT)
      return Intake(
        frontMotor,
        backMotor
      )
    }
  }
}
