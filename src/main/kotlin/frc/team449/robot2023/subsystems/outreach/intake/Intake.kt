package frc.team449.robot2023.subsystems.outreach.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Intake(
  private val intakeMotor: WrappedMotor
) : SubsystemBase() {

  // Run intake motor forwards.
  fun runIntakeForward() {
    intakeMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
  }

  // Run intake motor in reverse.
  fun runIntakeReverse() {
    intakeMotor.setVoltage(-IntakeConstants.INTAKE_VOLTAGE)
  }

  // Stop intake motor.
  fun stopIntake() {
    intakeMotor.setVoltage(0.0)
  }
}
