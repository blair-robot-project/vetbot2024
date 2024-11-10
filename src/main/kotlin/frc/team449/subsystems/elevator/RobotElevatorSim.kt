package frc.team449.subsystems.elevator

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.ChassisReference
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotController
import frc.team449.subsystems.RobotConstants
import java.util.function.Supplier

class RobotElevatorSim(
  private val motor: TalonFX
) : Elevator(motor) {
  private val sim = TiltedElevatorSim(
    DCMotor.getKrakenX60(1),
    ElevatorConstants.GEARING_MOTOR_TO_GEAR,
    ElevatorConstants.CARRIAGE_MASS_KG,
    ElevatorConstants.GEAR_DIAMETER_M / 2,
    0.0,
    ElevatorConstants.MAX_HEIGHT,
    false, // TODO: TURN THIS ON ONCE YOU FOUND KG
    angle = ElevatorConstants.ANGLE
  )
  private var simMotor = motor.simState

  override val positionSupplier = Supplier { sim.positionMeters }
  override val velocitySupplier = Supplier { sim.velocityMetersPerSecond }
  var currentDraw = 0.0

  override fun periodic() {
    super.periodic()
    simMotor = motor.simState
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage())

    simMotor.Orientation = ChassisReference.CounterClockwise_Positive

    sim.setInputVoltage(simMotor.motorVoltage)
    sim.update(RobotConstants.LOOP_TIME)
    currentDraw = sim.currentDrawAmps

    simMotor.setRawRotorPosition(sim.positionMeters * ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR)
    simMotor.setRotorVelocity(sim.velocityMetersPerSecond * ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR)
  }

  override fun initSendable(builder: SendableBuilder) {
    super.initSendable(builder)

    builder.publishConstString("4.0", "Sim Stuff")
    builder.addDoubleProperty("4.1 Current Draw", { currentDraw }, null)
  }
}
