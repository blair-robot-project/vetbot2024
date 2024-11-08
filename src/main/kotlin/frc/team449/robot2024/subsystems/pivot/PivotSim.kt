package frc.team449.robot2024.subsystems.pivot

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants
import java.util.function.Supplier

class PivotSim(
  private val motor: TalonFX
): Pivot(motor) {

  private val sim = SingleJointedArmSim(
    DCMotor.getKrakenX60(1),
    PivotConstants.GEARING_MOTOR_TO_MECHANISM,
    PivotConstants.MOMENT_OF_INERTIA,
    PivotConstants.PIVOT_LENGTH,
    PivotConstants.MIN_ANGLE,
    PivotConstants.MAX_ANGLE,
    true,
    PivotConstants.MIN_ANGLE
  )

  private var simMotor = motor.simState

  override val positionSupplier = Supplier { Radians.of(sim.angleRads) }
  override val velocitySupplier = Supplier { RadiansPerSecond.of(sim.velocityRadPerSec) }
  private var currentDraw = 0.0

  override fun periodic() {
    super.periodic()
    simMotor = motor.simState
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage())

    sim.setInputVoltage(MathUtil.clamp(simMotor.motorVoltage, -12.0, 12.0))
    sim.update(RobotConstants.LOOP_TIME)
    currentDraw = sim.currentDrawAmps

    simMotor.setRawRotorPosition(positionSupplier.get().`in`(Rotations) * PivotConstants.GEARING_MOTOR_TO_MECHANISM)
    simMotor.setRotorVelocity(velocitySupplier.get().`in`(RotationsPerSecond) * PivotConstants.GEARING_MOTOR_TO_MECHANISM)
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)

    builder.publishConstString("4.0", "Sim Stuff")
    builder.addDoubleProperty("4.1 Current Draw", {currentDraw}, null)

  }
}