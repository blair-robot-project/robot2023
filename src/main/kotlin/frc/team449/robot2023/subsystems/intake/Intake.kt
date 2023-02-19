package frc.team449.robot2023.subsystems.intake

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(
  private val intakePiston: DoubleSolenoid,
  private val sensor: DigitalInput
) : SubsystemBase() {

  private var sensorVal = false
  private var previousVal = false
  init {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun pistonOn() {
    intakePiston.set(DoubleSolenoid.Value.kForward)
  }

  fun pistonRev() {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
  }

  override fun periodic() {
    sensorVal = !sensor.get()
    if (sensorVal != previousVal && sensorVal) {
      pistonOn()
    }
    previousVal = sensorVal
  }
}
