package frc.team449.robot2023.subsystems.arm

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase

class ArmFollower(
  private val arm: Arm,
  private val trajectory: ArmTrajectory
) : CommandBase() {

  private val timer = Timer()
  private var prevTime = 0.0

  override fun initialize() {
    addRequirements(arm)
    timer.reset()
    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()

    val reference: ArmState = trajectory.sample(currTime)

    arm.state = reference

    prevTime = currTime
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
