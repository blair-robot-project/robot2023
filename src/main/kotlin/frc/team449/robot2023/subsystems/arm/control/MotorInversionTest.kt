package frc.team449.robot2023.subsystems.arm.control

import frc.team449.system.motor.WrappedMotor

class MotorInversionTest(
  private val otherMotor : WrappedMotor,
  private val motor : WrappedMotor
) {

  fun runMotorPositive() {
    motor.set(.1)
  }

  fun runOtherMotorPositive() {
    otherMotor.set(.1)
  }

  fun stopMotors() {
    motor.set(0.0)
    otherMotor.set(0.0)
  }
}