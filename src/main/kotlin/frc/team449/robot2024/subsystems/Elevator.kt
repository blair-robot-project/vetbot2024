package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.subsystem.ElevatorConstants

class Elevator(
  private val motor: TalonFX
) : SubsystemBase() {

  private val positionRequest = PositionVoltage(0.0)
    .withSlot(0)
    .withEnableFOC(false)
    .withUpdateFreqHz(50.0)

  init {
    name = "Elevator"
    motor.optimizeBusUtilization()

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()
  }

  fun moveToPosition(position: Double) {
    motor.setControl(positionRequest.withPosition(position))
  }

  fun setPosition(position: Double): Command {
    return this.runOnce { moveToPosition(position) }
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Stuff")
    builder.addDoubleProperty("1.1 Front Voltage", { motor.motorVoltage.value }, null)
    builder.addDoubleProperty("1.2 Front Position", { motor.position.value }, null)
    builder.addDoubleProperty("1.3 Front Velocity", { motor.velocity.value }, null)
    builder.addDoubleProperty("1.4 Desired Position", { positionRequest.Position }, null)
  }

  companion object {
    fun createElevator(): Elevator {
      val motor = TalonFX(ElevatorConstants.MOTOR_ID)
      val config = TalonFXConfiguration()
      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT
      config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT
      config.CurrentLimits.SupplyCurrentThreshold = ElevatorConstants.BURST_CURRENT_LIMIT
      config.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.BURST_TIME_LIMIT
      config.Slot0.kS = ElevatorConstants.RIGHT_KS
      config.Slot0.kV = ElevatorConstants.RIGHT_KV
      config.Slot0.kA = ElevatorConstants.RIGHT_KA
      config.Slot0.kP = ElevatorConstants.RIGHT_KP
      config.Slot0.kI = ElevatorConstants.RIGHT_KI
      config.Slot0.kD = ElevatorConstants.RIGHT_KD
      config.MotorOutput.Inverted = ElevatorConstants.ORIENTATION
      config.MotorOutput.NeutralMode = ElevatorConstants.NEUTRAL_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = ElevatorConstants.DUTY_CYCLE_DEADBAND
      config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARING
      motor.configurator.apply(config)
      motor.velocity.setUpdateFrequency(ElevatorConstants.UPDATE_FREQUENCY)
      motor.motorVoltage.setUpdateFrequency(ElevatorConstants.UPDATE_FREQUENCY)
      motor.closedLoopError.setUpdateFrequency(ElevatorConstants.UPDATE_FREQUENCY)
      motor.optimizeBusUtilization()

      return Elevator(motor)
    }
  }
}
