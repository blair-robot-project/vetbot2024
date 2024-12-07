package frc.team449.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import java.util.function.Supplier
import kotlin.math.abs

open class Elevator(
  private val motor: TalonFX
) : SubsystemBase() {

  open val positionSupplier = Supplier { motor.position.value }
  open val velocitySupplier = Supplier { motor.velocity.value }
  open val targetSupplier = Supplier { request.Position }

  private val timer = Timer()

  // simulation
  private val mech = Mechanism2d(1.5, 2.0)
  private val rootElevator: MechanismRoot2d = mech.getRoot("elevator", 0.25, 0.25)
  private val elevatorVisual: MechanismLigament2d = rootElevator.append(
    MechanismLigament2d(
      "elevator",
      ElevatorConstants.MIN_HEIGHT,
      ElevatorConstants.ANGLE,
      ElevatorConstants.WIDTH,
      ElevatorConstants.COLOR
    )
  )

  private val rootDesiredElevator: MechanismRoot2d = mech.getRoot("desiredElevator", 0.25, 0.25)
  private val desiredElevatorVisual: MechanismLigament2d = rootDesiredElevator.append(
    MechanismLigament2d(
      "desiredElevator",
      ElevatorConstants.MIN_HEIGHT,
      ElevatorConstants.ANGLE,
      ElevatorConstants.TARGET_WIDTH,
      ElevatorConstants.DESIRED_COLOR
    )
  )

  private val request = MotionMagicVoltage(ElevatorConstants.STOW_HEIGHT)
    .withSlot(0)
    .withEnableFOC(false)
    .withUpdateFreqHz(100.0)

  fun setPosition(position: Double): Command {
    return this.runOnce { motor.setControl(request.withPosition(position)) }
  }

  fun setVoltage(voltage: Double) {
    motor.setControl(
      VoltageOut(
        voltage,
        false,
        false,
        false,
        false
      )
    )
  }

  fun currentHoming(): Command {
    return FunctionalCommand(
      {
        timer.stop()
        timer.reset()
      },
      {
        setVoltage(ElevatorConstants.HOMING_VOLTAGE.`in`(Volts))
        if (motor.statorCurrent.value > ElevatorConstants.HOMING_CURRENT_CUTOFF.`in`(Amps) &&
          motor.velocity.value < ElevatorConstants.HOMING_MAX_VEL.`in`(MetersPerSecond)
        ) {
          timer.start()
        } else {
          timer.stop()
          timer.reset()
        }
      },
      {
        motor.setPosition(ElevatorConstants.STOW_HEIGHT)
        println("COMPLETED ELEVATOR CURRENT HOMING, SET TO STOW POSITION")
      },
      {
        motor.statorCurrent.value > ElevatorConstants.HOMING_CURRENT_CUTOFF.`in`(Amps) &&
          timer.hasElapsed(ElevatorConstants.HOMING_TIME_CUTOFF.`in`(Seconds))
      }
    )
      .andThen(stow())
  }

  fun manualDown(): Command {
    return runOnce{setVoltage(-3.0)}
  }

  fun manualUp(): Command {
    return runOnce{setVoltage(3.0)}
  }

  fun hold(): Command {
    return setPosition(targetSupplier.get())
  }

  fun stop(): Command {
    return this.run { motor.stopMotor() }
  }

  fun stow(): Command {
    return setPosition(ElevatorConstants.STOW_HEIGHT)
  }

  fun high(): Command {
    return setPosition(ElevatorConstants.HIGH_HEIGHT)
  }

  fun atSetpoint(): Boolean {
    return (abs(motor.position.value - request.Position) < ElevatorConstants.TOLERANCE)
  }

  override fun periodic() {
    elevatorVisual.length = ElevatorConstants.MIN_HEIGHT + positionSupplier.get()
    desiredElevatorVisual.length = ElevatorConstants.MIN_HEIGHT + request.Position

    SmartDashboard.putData("Elevator Visual", mech)
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Elevator Info")
    builder.addDoubleProperty("1.1 Voltage", { motor.motorVoltage.value }, null)
    builder.addDoubleProperty("1.2 Position", { positionSupplier.get() }, null)
    builder.addDoubleProperty("1.3 Velocity", { velocitySupplier.get() }, null)
    builder.addDoubleProperty("1.4 Desired Position", { request.Position }, null)
    builder.addDoubleProperty("1.5 Closed-loop Error", { motor.closedLoopError.value }, null)
    builder.addBooleanProperty("1.6 At Tolerance", { atSetpoint() }, null)
    //builder.addStringProperty("1.7 Command", {this.currentCommand.name}, null)
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

      config.Slot0.kS = ElevatorConstants.KS
      config.Slot0.kV = ElevatorConstants.KV
      config.Slot0.kA = ElevatorConstants.KA
      config.Slot0.kP = ElevatorConstants.KP
      config.Slot0.kI = ElevatorConstants.KI
      config.Slot0.kD = ElevatorConstants.KD
      config.Slot0.kG = ElevatorConstants.KG
      config.Slot0.GravityType = GravityTypeValue.Elevator_Static

      config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MM_ACCEL
      config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MM_VEL

      config.MotorOutput.Inverted = ElevatorConstants.ORIENTATION
      config.MotorOutput.NeutralMode = ElevatorConstants.NEUTRAL_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = ElevatorConstants.DUTY_CYCLE_DEADBAND
      config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR

      motor.configurator.apply(config)

      BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.UPDATE_FREQUENCY,
        motor.position,
        motor.velocity,
        motor.motorVoltage,
      )

      motor.optimizeBusUtilization()

      return if (RobotBase.isReal()) Elevator(motor) else RobotElevatorSim(motor)
    }
  }
}
