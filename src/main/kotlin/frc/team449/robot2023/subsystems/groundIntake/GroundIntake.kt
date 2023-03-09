package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.endEffector.EndEffector
import frc.team449.system.motor.WrappedMotor

class GroundIntake(
  private val intakeMotor: WrappedMotor,
  private val intakePiston: DoubleSolenoid,
  private val arm: Arm,
  private val endEffector: EndEffector
) : SubsystemBase() {

  private var retracted = true

  fun runIntake() {
    intakeMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun runIntakeReverse() {
    intakeMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun deploy() {
    intakePiston.set(DoubleSolenoid.Value.kForward)
    retracted = false
  }

  fun retract() {
    intakePiston.set(DoubleSolenoid.Value.kReverse)
    retracted = true
  }

  fun toggleRollerState() {
    if (retracted) {
      deploy()
    }
    else {
      retract()
    }
  }

  fun handoff(): Command {
    return if (arm.state == ArmConstants.STOW && this.retracted) {
      InstantCommand(this::runIntakeReverse).andThen(
        WaitCommand(0.1)
      ).andThen(
        endEffector::pistonOn
      )
    }
    else {
      InstantCommand()
    }
  }

  fun stop() {
    intakeMotor.stopMotor()
  }
}
