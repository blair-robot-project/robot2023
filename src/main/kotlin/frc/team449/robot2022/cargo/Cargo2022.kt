package frc.team449.robot2022.cargo

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable

class Cargo2022(
  val intakeMotor: WrappedMotor,
  val spitMotor: WrappedMotor,
  val shootMotor: WrappedMotor,
) : SubsystemBase(), Loggable {

  fun runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT)
    spitMotor.set(CargoConstants.SPITTER_INTAKE_SPEED_RPS)
  }

  fun runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT)
  }

  fun shoot() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT)
    spitMotor.set(CargoConstants.SPITTER_SHOOT_SPEED_RPS)
    shootMotor.set(CargoConstants.SHOOTER_SPEED_RPS)
  }

  fun stop() {
    intakeMotor.set(0.0)
    spitMotor.set(0.0)
    shootMotor.set(0.0)
  }
}
