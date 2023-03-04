package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class GroundIntake(
  private val leftMotor: WrappedMotor,
  private val rightMotor: WrappedMotor,
) : SubsystemBase() {
  fun stop() {
    leftMotor.set(0.0)
    rightMotor.set(0.0)
  }

  fun run() {
    leftMotor.set(0.1)
    rightMotor.set(0.1)
  }

  fun runReverse() {
    leftMotor.set(-0.05)
    rightMotor.set(-0.05)
  }
}
