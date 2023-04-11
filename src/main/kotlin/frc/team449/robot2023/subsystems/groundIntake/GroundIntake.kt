package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.robot2023.constants.subsystem.GroundIntakeConstants
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.WrappedMotor
import frc.team449.system.motor.createSparkMax

class GroundIntake(
  private val topMotor: WrappedMotor,
  private val bottomMotor: WrappedMotor,
  private val piston: DoubleSolenoid
) : SubsystemBase() {

  init {
    this.retract()
    this.stop()
  }

  private var retracted = true

  fun intakeCone(): Command {
    return this.runOnce {
      topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
      bottomMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun intakeCube(): Command {
    return this.runOnce {
      topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
      bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun outtake(): Command {
    return this.runOnce {
      topMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
      bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
    }
  }

  fun deploy(): Command {
    return this.runOnce {
      piston.set(DoubleSolenoid.Value.kForward)
      retracted = false
    }
  }

  fun retract(): Command {
    return this.runOnce {
      piston.set(DoubleSolenoid.Value.kReverse)
      retracted = true
    }
  }

  fun scoreLow(): Command {
    return deploy()
      .andThen(WaitCommand(.45))
      .andThen(outtake())
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
        currentLimit = GroundIntakeConstants.CURRENT_LIM
      )

      val bottomMotor = createSparkMax(
        "Intake Bottom",
        GroundIntakeConstants.INTAKE_BOTTOM,
        NEOEncoder.creator(
          GroundIntakeConstants.UPR,
          GroundIntakeConstants.GEARING
        ),
        inverted = GroundIntakeConstants.BOTTOM_INVERTED,
        currentLimit = GroundIntakeConstants.CURRENT_LIM
      )

      // create ground intake pistons
      val piston = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.PISTON_FWD,
        GroundIntakeConstants.PISTON_REV
      )

      return GroundIntake(
        topMotor,
        bottomMotor,
        piston
      )
    }
  }
}
