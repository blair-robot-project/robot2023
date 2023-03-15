package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.system.motor.WrappedMotor

class GroundIntake(
  private val piston: DoubleSolenoid,
  private val leftMotor: WrappedMotor,
  private val rightMotor: WrappedMotor
) : SubsystemBase() {

  private var retracted = true

  fun runIntake() {
    leftMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    rightMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun runIntakeReverse() {
    leftMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
    rightMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun deploy() {
    piston.set(DoubleSolenoid.Value.kForward)
    retracted = false
  }

  fun retract() {
    piston.set(DoubleSolenoid.Value.kReverse)
    retracted = true
  }

  fun stop() {
    leftMotor.setVoltage(0.0)
    rightMotor.setVoltage(0.0)
  }
}
