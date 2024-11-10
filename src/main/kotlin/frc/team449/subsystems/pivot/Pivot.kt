package frc.team449.subsystems.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.Supplier
import kotlin.math.abs

open class Pivot(
  private val motor: TalonFX
) : SubsystemBase() {

  private val positionRequest = MotionMagicVoltage(PivotConstants.STOW_ANGLE.`in`(Rotations))
    .withSlot(0)
    .withEnableFOC(false)
    .withUpdateFreqHz(1000.0)

  open val positionSupplier: Supplier<Measure<Angle>> = Supplier { Rotations.of(motor.position.value) }
  open val velocitySupplier: Supplier<Measure<Velocity<Angle>>> = Supplier { RotationsPerSecond.of(motor.velocity.value) }
  open var target: Measure<Angle> = PivotConstants.STOW_ANGLE

  // sim stuff
  private val mech = Mechanism2d(2.0, 2.0)
  private val pivotRoot = mech.getRoot("pivot", 0.25, 0.25)
  private val pivotVisual = pivotRoot.append(
    MechanismLigament2d(
      "pivot",
      PivotConstants.PIVOT_LENGTH,
      PivotConstants.STOW_ANGLE.`in`(Degrees),
      PivotConstants.WIDTH,
      PivotConstants.REAL_COLOR
    )
  )

  private val targetRoot = mech.getRoot("pivot", 0.25, 0.25)
  private val targetVisual = targetRoot.append(
    MechanismLigament2d(
      "pivot target",
      PivotConstants.PIVOT_LENGTH,
      PivotConstants.STOW_ANGLE.`in`(Degrees),
      PivotConstants.TARGET_WIDTH,
      PivotConstants.TARGET_COLOR
    )
  )

  fun setPosition(rotations: Measure<Angle>): Command {
    return this.runOnce {
      target = rotations
      motor.setControl(positionRequest.withPosition(rotations.`in`(Rotations)))
    }
  }

  fun setVoltage(voltage: Measure<Voltage>) {
    motor.setControl(
      VoltageOut(
        voltage.`in`(Volts),
        false,
        false,
        false,
        false
      )
    )
  }

  fun calibrateStartingPos() {
    motor.setPosition(PivotConstants.STOW_ANGLE.`in`(Rotations))
  }

  fun stow(): Command {
    return setPosition(PivotConstants.STOW_ANGLE)
  }

  fun intakeAngle(): Command {
    return setPosition(PivotConstants.INTAKE_ANGLE)
  }

  fun high(): Command {
    return setPosition(PivotConstants.HIGH_ANGLE)
  }

  fun atSetpoint(): Boolean {
    return abs(positionSupplier.get().`in`(Rotations) - target.`in`(Rotations)) <
      PivotConstants.TOLERANCE.`in`(Rotations)
  }

  override fun periodic() {
    targetVisual.angle = target.`in`(Degrees)
    pivotVisual.angle = positionSupplier.get().`in`(Degrees)

    SmartDashboard.putData("pivot visual", mech)
  }

  // logging stuff
  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Stuff")
    builder.addDoubleProperty("1.1 Voltage V", { motor.motorVoltage.value }, null)
    builder.addDoubleProperty("1.2 Velocity RPS", { velocitySupplier.get().`in`(RotationsPerSecond) }, null)
    builder.addDoubleProperty("1.3 Current Position Rot", { positionSupplier.get().`in`(Rotations) }, null)
    builder.addDoubleProperty("1.4 Desired Position Rot", { positionRequest.Position }, null)
    builder.addDoubleProperty("1.5 Stator Current A", { motor.statorCurrent.value }, null)
    builder.publishConstString("2.0", "Model info")
    builder.addDoubleProperty("2.1 Closed Loop Error Rot", { motor.closedLoopError.value }, null)
    builder.addBooleanProperty("2.2 At Setpoint", { atSetpoint() }, null)
  }

  companion object {
    fun createPivot(): Pivot {
      val motor = TalonFX(PivotConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_CURRENT_LIMIT
      config.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT
      config.CurrentLimits.SupplyCurrentThreshold = PivotConstants.BURST_CURRENT_LIMIT
      config.CurrentLimits.SupplyTimeThreshold = PivotConstants.BURST_TIME_LIMIT

      config.Slot0.kS = PivotConstants.KS
      config.Slot0.kV = PivotConstants.KV
      config.Slot0.kA = PivotConstants.KA
      config.Slot0.kP = PivotConstants.KP
      config.Slot0.kI = PivotConstants.KI
      config.Slot0.kD = PivotConstants.KD
      config.Slot0.kG = PivotConstants.KG
      config.Slot0.GravityType = GravityTypeValue.Arm_Cosine

      config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VEL.`in`(RadiansPerSecond)
      config.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCEL.`in`(RadiansPerSecond.per(Second))

      config.MotorOutput.Inverted = PivotConstants.ORIENTATION
      config.MotorOutput.NeutralMode = PivotConstants.NEUTRAL_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = PivotConstants.DUTY_CYCLE_DEADBAND
      config.Feedback.SensorToMechanismRatio = PivotConstants.GEARING_MOTOR_TO_MECHANISM

      motor.configurator.apply(config)

      BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.UPDATE_FREQUENCY,
        motor.position,
        motor.velocity,
        motor.motorVoltage,
        motor.supplyCurrent,
        motor.torqueCurrent,
        motor.deviceTemp
      )
      motor.optimizeBusUtilization()

      motor.setPosition(PivotConstants.STOW_ANGLE.`in`(Rotations))

      return if (RobotBase.isReal()) Pivot(motor) else PivotSim(motor)
    }
  }
}
