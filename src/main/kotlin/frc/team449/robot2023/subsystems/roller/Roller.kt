package frc.team449.robot2023.subsystems.roller

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.system.motor.WrappedMotor

class Roller(
  private val rollerMotor: WrappedMotor,
  private val arm: Arm
) : SubsystemBase() {

  fun runIntake() {
    rollerMotor.setVoltage(RollerConstants.INTAKE_VOLTAGE)
  }

  fun runIntakeReverse() {
    rollerMotor.setVoltage(-RollerConstants.INTAKE_VOLTAGE)
  }

  fun stop() {
    rollerMotor.stopMotor()
  }
}
