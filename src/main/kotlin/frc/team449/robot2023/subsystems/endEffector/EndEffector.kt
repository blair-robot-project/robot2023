package frc.team449.robot2023.subsystems.endEffector

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

class EndEffector(
  private val intakePiston: DoubleSolenoid,
  private val sensor: DigitalInput
) : SubsystemBase(), Sendable {
  private var sensorVal = false
  private var previousVal = false

  init {
    this.pistonRev()
  }

  fun pistonOn() {
    intakePiston.set(DoubleSolenoid.Value.kForward)
  }

  fun pistonRev() {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
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
    builder.addStringProperty("piston", { intakePiston.get().name }, null)
  }
}
