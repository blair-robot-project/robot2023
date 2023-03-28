package frc.team449.robot2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm

class ArmMove(
  private val arm: Arm
) : CommandBase() {

  init {
    addRequirements(arm)
  }

  override fun execute() {
    val ff = arm.getArmFeedForward().calculate(arm.desiredState.matrix, false)
    val pid = arm.getArmPDController().calculate(arm.state.matrix, arm.desiredState.matrix)
    val u = ff + pid
    arm.firstJoint.setVoltage(u[0, 0])
    arm.secondJoint.setVoltage(u[1, 0])
    arm.visual.setState(arm.state, arm.desiredState)
  }
}