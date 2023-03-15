package frc.team449.robot2023.subsystems.endEffector

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.constants.subsystem.EndEffectorConstants
import frc.team449.system.motor.WrappedMotor

class RollerIntake(
  private val intakeMotor: WrappedMotor,
  private val chooserPiston: DoubleSolenoid,
  private val sensor: DigitalInput
) : SubsystemBase() {
  private var sensorVal = false

  // forward is cone, reverse is cube
  fun readyCube() {
    chooserPiston.set(DoubleSolenoid.Value.kReverse)
  }

  fun readyCone() {
    chooserPiston.set(DoubleSolenoid.Value.kForward)
  }

  fun intake() {
    intakeMotor.setVoltage(EndEffectorConstants.INTAKE_VOLTAGE)
  }

  fun intakeSlow() {
    intakeMotor.setVoltage(EndEffectorConstants.INTAKE_VOLTAGE * EndEffectorConstants.SLOW_MULTIPLIER)
  }

  fun intakeReverse() {
    intakeMotor.setVoltage(EndEffectorConstants.INTAKE_VOLTAGE)
  }

  fun stop() {
    intakeMotor.set(0.0)
  }
}
