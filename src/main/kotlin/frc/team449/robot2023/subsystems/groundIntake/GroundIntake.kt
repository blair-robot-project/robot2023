package frc.team449.robot2023.subsystems.groundIntake

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
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
import java.time.Instant

class GroundIntake(
  private val topMotor: WrappedMotor,
  private val bottomMotor: WrappedMotor,
  private val intakePiston: DoubleSolenoid
) : SubsystemBase() {

  init {
    this.retract()
    this.stop()
  }

  private var retracted = true

  fun intakeCone(): Command {
    return InstantCommand(
      {
        topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
        bottomMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
      }
    )
  }

  fun intakeCube(): Command {
    return InstantCommand(
      {
        topMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
        bottomMotor.setVoltage(GroundIntakeConstants.INTAKE_VOLTAGE)
      }
    )
  }

  fun outtake(): Command {
    return InstantCommand(
      {
        topMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
        bottomMotor.setVoltage(-GroundIntakeConstants.INTAKE_VOLTAGE)
      }
    )
  }

  fun deploy(): Command {
    retracted = false
    return InstantCommand(
      {
        intakePiston.set(DoubleSolenoid.Value.kForward)
      }
    )
  }

  fun retract(): Command {
    retracted = true
    return InstantCommand(
      {
        intakePiston.set(DoubleSolenoid.Value.kReverse)
      }
    )
  }

  fun stop(): Command {
    return InstantCommand(
      {
        topMotor.stopMotor()
        bottomMotor.stopMotor()
      }
    )
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
      val piston = DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        GroundIntakeConstants.FWD_CHANNEL,
        GroundIntakeConstants.REV_CHANNEL
      )

      return GroundIntake(
        topMotor,
        bottomMotor,
        piston
      )
    }
  }
}
