package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.system.motor.WrappedMotor

class GroundIntake(
  private val intakeMotor: WrappedMotor,
  private val intakePiston: DoubleSolenoid,
  private val arm: Arm
) : SubsystemBase() {

  private val retracted = true

  fun runIntake() {
    intakeMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun runIntakeReverse() {
    intakeMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun deploy() {
    intakePiston.set(DoubleSolenoid.Value.kForward)
  }

  fun retract() {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun toggleRollerState() {
    if (retracted) {
      deploy()
    }
    else {
      retract()
    }
  }

  fun stop() {
    intakeMotor.stopMotor()
  }
}
