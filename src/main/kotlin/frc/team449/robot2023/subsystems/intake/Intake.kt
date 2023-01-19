package frc.team449.robot2023.subsystems.intake

import edu.wpi.first.wpilibj.DoubleSolenoid

class Intake(
  private val intakePiston: DoubleSolenoid
) {

  init {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun pistonOn() {
    intakePiston.set(DoubleSolenoid.Value.kForward)
  }

  fun pistonRev() {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
  }
}
