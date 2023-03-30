package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class GroundIntake(
  private val topMotor: WrappedMotor,
  private val bottomMotor: WrappedMotor,
  private val intakePiston1: DoubleSolenoid,
  private val intakePiston2: DoubleSolenoid,
) : SubsystemBase() {

  init {
    this.retract()
    this.stop()
  }

  private var retracted = true

  fun intakeCone() {
    topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    bottomMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun intakeCube() {
    topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
  }

  fun outtake() {
    topMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
    bottomMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
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
    fun createGroundIntake(): GroundIntake {
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
        GroundIntakeConstants.PISTON_FWD_1,
        GroundIntakeConstants.PISTON_REV_1
      )

      val piston2 = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.PISTON_FWD_2,
        GroundIntakeConstants.PISTON_REV_2
      )

      return GroundIntake(
        topMotor,
        bottomMotor,
        piston1,
        piston2,
      )
    }
  }
}
