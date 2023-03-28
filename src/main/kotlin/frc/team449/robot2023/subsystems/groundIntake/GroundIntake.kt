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
  private val topMotor: WrappedMotor,
  private val bottomMotor: WrappedMotor,
  private val intakePiston1: DoubleSolenoid,
  private val intakePiston2: DoubleSolenoid,
  private val arm: Arm,
  private val endEffector: EndEffector
) : SubsystemBase() {

  init {
    this.retract()
    this.stop()
  }

  private var retracted = true

  fun intakeCone() {
    topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE * -1)
  }

  fun intakeCube() {
    topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun outtake() {
    topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE * -1)
    bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE * -1)
  }

  fun deploy() {
    intakePiston1.set(DoubleSolenoid.Value.kForward)
    intakePiston2.set(DoubleSolenoid.Value.kForward)
    retracted = false
  }

  fun retract() {
    intakePiston1.set(DoubleSolenoid.Value.kForward)
    intakePiston2.set(DoubleSolenoid.Value.kForward)
    retracted = true
  }

  fun handoff(): Command {
    return if (arm.desiredState == ArmConstants.STOW && this.retracted) {
//      InstantCommand({ intakeMotor.setVoltage(-1.0) }).andThen(
//        WaitCommand(0.1)
//      ).andThen(
//        endEffector::pistonOn
//      )
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
      InstantCommand()
    } else {
      InstantCommand()
    }
  }

  fun scoreLow(): Command {
    return InstantCommand(::deploy)
      .andThen(WaitCommand(.45))
      .andThen(::outtake)
  }
  fun stop() {
    topMotor.stopMotor()
    bottomMotor.stopMotor()
  }

  companion object {
    fun createGroundIntake(robot: Robot): GroundIntake {
      val topMotor = createSparkMax(
        "Intake Top",
        GroundIntakeConstants.INTAKE_TOP,
        NEOEncoder.creator(
          GroundIntakeConstants.UPR,
          GroundIntakeConstants.GEARING
        ),
        inverted = GroundIntakeConstants.TOP_INVERTED,
        currentLimit = GroundIntakeConstants.CURRENT_LIM,
      )

      val bottomMotor = createSparkMax(
        "Intake Bottom",
        GroundIntakeConstants.INTAKE_BOTTOM,
        NEOEncoder.creator(
          GroundIntakeConstants.UPR,
          GroundIntakeConstants.GEARING
        ),
        inverted = GroundIntakeConstants.BOTTOM_INVERTED,
        currentLimit = GroundIntakeConstants.CURRENT_LIM,
      )

      // create ground intake pistons
      val piston1 = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.PISTON_FWD,
        GroundIntakeConstants.PISTON_REV
      )

      val piston2 = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.PISTON_FWD,
        GroundIntakeConstants.PISTON_REV
      )

      return GroundIntake(
        topMotor,
        bottomMotor,
        piston1,
        piston2,
        robot.arm,
        robot.endEffector
      )
    }
  }
}
