package frc.team449.robot2023.subsystems.arm.control

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.arm.Arm

open class ArmFollower(
  private val arm: Arm,
  private val trajectory: ArmTrajectory
) : CommandBase() {

  val timer = Timer()

  override fun initialize() {
    addRequirements(arm)
    timer.reset()
    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()

    val reference: ArmState = trajectory.sample(currTime)

    arm.state = reference
  }

  override fun isFinished(): Boolean {
    return timer.get() > trajectory.totalTime
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    arm.stop()
  }
}
