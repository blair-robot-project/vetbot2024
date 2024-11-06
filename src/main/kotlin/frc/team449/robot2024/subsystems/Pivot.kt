package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.subsystem.PivotConstants

class Pivot(
  private val motor: TalonFX,
  private val loop: LinearSystemLoop<N2, N1, N1>
) : SubsystemBase() {

  private val positionRequest = MotionMagicVoltage(PivotConstants.START_ANGLE)
    .withSlot(0)
    .withEnableFOC(false)
    .withUpdateFreqHz(1000.0)

  // sim stuff
  private val mech = Mechanism2d(2.0, 2.0)
  private val pivotRoot = mech.getRoot("pivot", 0.25, 0.25)
  private val pivotVisual = pivotRoot.append(
    MechanismLigament2d(
      "pivot",
      PivotConstants.PIVOT_LENGTH,
      PivotConstants.START_ANGLE,
      PivotConstants.WIDTH,
      PivotConstants.REAL_COLOR
    )
  )

  private val targetRoot = mech.getRoot("pivot", 0.25, 0.25)
  private val targetVisual = targetRoot.append(
    MechanismLigament2d(
      "pivot target",
      PivotConstants.PIVOT_LENGTH,
      PivotConstants.START_ANGLE,
      PivotConstants.WIDTH,
      PivotConstants.TARGET_COLOR
    )
  )

  fun moveToAngle(rotations: Double): Command {
    loop.nextR = VecBuilder.fill(rotations, 0.0)
    return this.run {
      loop.correct(VecBuilder.fill(motor.position.value))
      loop.predict(RobotConstants.LOOP_TIME)
      val nextVoltage = loop.getU(0)
      motor.setVoltage(nextVoltage)
    }
  }

  fun setPosition(position: Double): Command {
    return this.runOnce { motor.setControl(positionRequest.withPosition(position)) }
  }

  override fun periodic() {
    pivotVisual.angle = Units.rotationsToDegrees(motor.position.value)
    targetVisual.angle = Units.rotationsToDegrees(positionRequest.Position)

    SmartDashboard.putData("pivot visual", mech)
  }

  // logging stuff
  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Motor Stuff")
    builder.addDoubleProperty("1.1 Voltage", { motor.motorVoltage.value }, null)
    builder.addDoubleProperty("1.2 Velocity", { motor.velocity.value }, null)
    builder.addDoubleProperty("1.3 Current Position", { motor.position.value }, null)
    builder.addDoubleProperty("1.4 Desired Position", { positionRequest.Position }, null)
    builder.addDoubleProperty("1.5 Stator Current", { motor.statorCurrent.value }, null)
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

      config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VEL
      config.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCEL

      config.MotorOutput.Inverted = PivotConstants.ORIENTATION
      config.MotorOutput.NeutralMode = PivotConstants.NEUTRAL_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = PivotConstants.DUTY_CYCLE_DEADBAND
      config.Feedback.SensorToMechanismRatio = PivotConstants.GEARING

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

      val plant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getKrakenX60(1),
        PivotConstants.MOMENT_OF_INERTIA,
        PivotConstants.GEARING
      )

      val filter = KalmanFilter(
        Nat.N2(),
        Nat.N1(),
        plant,
        VecBuilder.fill(
          PivotConstants.MODEL_POS_DEVIATION,
          PivotConstants.MODEL_VEL_DEVIATION
        ),
        VecBuilder.fill(
          PivotConstants.ENCODER_POS_DEVIATION
        ),
        RobotConstants.LOOP_TIME
      )

      val controller = LinearQuadraticRegulator(
        plant,
        VecBuilder.fill(
          PivotConstants.POS_TOLERANCE,
          PivotConstants.VEL_TOLERANCE
        ),
        VecBuilder.fill(
          PivotConstants.CONTROL_EFFORT_VOLTS
        ),
        RobotConstants.LOOP_TIME
      )

      val loop = LinearSystemLoop(
        plant,
        controller,
        filter,
        PivotConstants.CONTROL_EFFORT_VOLTS,
        RobotConstants.LOOP_TIME
      )

      return Pivot(motor, loop)
    }
  }
}
