package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.endEffector.EndEffector
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class GroundIntake(
  private val intakeMotor: WrappedMotor,
  private val intakePiston: DoubleSolenoid,
  private val arm: Arm,
  private val endEffector: EndEffector
) : SubsystemBase() {

  init {
    this.retract()
    this.stop()
  }

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
    } else {
      retract()
    }
  }

  fun handoff(): Command {
    return if (arm.desiredState == ArmConstants.STOW && this.retracted) {
      InstantCommand({ intakeMotor.setVoltage(-1.0) }).andThen(
        WaitCommand(0.1)
      ).andThen(
        endEffector::pistonOn
      )
//      InstantCommand({ intakeMotor.encoder.resetPosition(0.0) }).andThen(
//        PIDCommand(
//          PIDController(4.0, 0.0, 0.0),
//          { intakeMotor.encoder.position },
//          1.34, // this is a measured pos
//          { x: Double -> intakeMotor.set(-x) }
//        ).withTimeout(1.0).andThen(
//          endEffector::pistonOn
//        )
//      )
    } else {
      InstantCommand()
    }
  }

  fun scoreLow(): Command {
    return InstantCommand(::deploy)
      .andThen(WaitCommand(.45))
      .andThen(::runIntakeReverse)
  }
  fun stop() {
    intakeMotor.stopMotor()
  }

  companion object {
    fun createGroundIntake(robot: Robot): GroundIntake {
      val groundIntakeMotor = createSparkMax(
        "GroundIntake",
        GroundIntakeConstants.INTAKE_RIGHT,
        NEOEncoder.creator(
          GroundIntakeConstants.UPR,
          GroundIntakeConstants.GEARING
        ),
        inverted = GroundIntakeConstants.INVERTED,
        currentLimit = GroundIntakeConstants.CURRENT_LIM,
        slaveSparks = mapOf(
          GroundIntakeConstants.INTAKE_LEFT to true
        )
      )

      // create ground intake pistons
      val groundIntakePiston = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.PISTON_FWD,
        GroundIntakeConstants.PISTON_REV
      )

      return GroundIntake(
        groundIntakeMotor,
        groundIntakePiston,
        robot.arm,
        robot.endEffector
      )
    }
  }
}
