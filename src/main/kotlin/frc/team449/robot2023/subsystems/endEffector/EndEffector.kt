package frc.team449.robot2023.subsystems.endEffector

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.EndEffectorConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class EndEffector(
  private val intakeMotor: WrappedMotor,
  val chooserPiston: DoubleSolenoid,
  private val sensor: DigitalInput
) : SubsystemBase() {
  private var sensorVal = false
  private var previousVal = false

  // forward is cone, reverse is cube
  fun pistonOn() {
    chooserPiston.set(DoubleSolenoid.Value.kForward)
  }

  fun pistonRev() {
    chooserPiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun intake() {
    intakeMotor.setVoltage(EndEffectorConstants.INTAKE_VOLTAGE)
  }

  fun holdIntake() {
    intakeMotor.setVoltage(EndEffectorConstants.HOLD_VOLTAGE)
  }

  fun intakeReverse() {
    intakeMotor.setVoltage(EndEffectorConstants.REVERSE_INTAKE_VOLTAGE)
  }

  fun stop() {
    intakeMotor.set(0.0)
  }

  fun autoReverse() {
    intakeMotor.setVoltage(-0.75)
  }

  override fun periodic() {
//    sensorVal = !sensor.get()
//    if (sensorVal != previousVal && sensorVal) {
//      pistonOn()
//    }
    previousVal = sensorVal
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.addBooleanProperty("sensor", { sensor.get() }, null)
    builder.addStringProperty("piston", { chooserPiston.get().toString() }, null)
    builder.addStringProperty("motor", { intakeMotor.get().toString() }, null)
  }

  companion object {
    fun createEndEffector(): EndEffector {
      val piston = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        EndEffectorConstants.FORWARD_CHANNEL,
        EndEffectorConstants.REVERSE_CHANNEL
      )

      val motor = createSparkMax(
        "End Effector Motor",
        EndEffectorConstants.MOTOR_ID,
        NEOEncoder.creator(
          EndEffectorConstants.MOTOR_UPR,
          EndEffectorConstants.MOTOR_GEARING
        ),
        inverted = false,
        currentLimit = EndEffectorConstants.MOTOR_CURR_LIM,
      )

      val sensor = DigitalInput(
        EndEffectorConstants.SENSOR_CHANNEL
      )

      return EndEffector(
        motor,
        piston,
        sensor
      )
    }
  }
}
