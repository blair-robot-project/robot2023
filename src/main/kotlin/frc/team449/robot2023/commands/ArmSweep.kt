package frc.team449.robot2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.Arm
import frc.team449.robot2023.subsystems.arm.control.ArmKinematics
import frc.team449.robot2023.subsystems.arm.control.CartesianArmState

class ArmSweep(
  private val arm: Arm,
  private val target: Double,
  private val PID: ProfiledPIDController,
  private val tolerance: Double
) : CommandBase() {

  private val kinematics = ArmKinematics(ArmConstants.LENGTH_1, ArmConstants.LENGTH_2)

  init {
    addRequirements(arm)

    PID.setTolerance(tolerance)

    PID.setGoal(target)
  }

  override fun execute() {
    arm.desiredState = kinematics.toAngularState(
      CartesianArmState(
        PID.calculate(arm.coordinate.x),
        arm.coordinate.z,
        0.0,
        0.0
      ),
      arm.state
    )!!
  }

  override fun isFinished(): Boolean {
    return PID.atGoal()
  }

  override fun end(interrupted: Boolean) {
    arm.stop()
  }
}
