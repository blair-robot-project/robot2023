package frc.team449.robot2023.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.control.ArmState

class ArmSweep(
  private val arm: Arm,
  private val input: () -> Double,
  private var sweepBeta: Rotation2d
) : CommandBase() {

  private lateinit var startState: ArmState

  init {
    addRequirements(arm)
  }

  override fun initialize() {
    startState = arm.desiredState.copy()
    if (startState.beta.degrees <= 0.0) sweepBeta = Rotation2d.fromDegrees(-sweepBeta.degrees)
    else sweepBeta = Rotation2d.fromDegrees(sweepBeta.degrees)
  }
  override fun execute() {
    arm.desiredState.beta = startState.beta + sweepBeta * input()
  }

  override fun isFinished(): Boolean {
    return false
  }
}
