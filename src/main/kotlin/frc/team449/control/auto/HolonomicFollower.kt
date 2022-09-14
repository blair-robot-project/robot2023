package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants

class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: PathPlannerTrajectory,
  private val resetPose: Boolean
) : CommandBase() {
  private val timer = Timer()
  private var prevTime = 0.0

  private var xController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var yController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var thetaController = PIDController(AutoConstants.ROTATION_KP, .0, .0)

  override fun initialize() {
    timer.start()
    if (resetPose) {
      drivetrain.pose = trajectory.initialPose
    }
  }

  override fun execute() {
    val currTime = timer.get()
    val dt = timer.get() - prevTime
    val desiredState = trajectory.sample(currTime)
    val currentState = drivetrain.pose
    // Figure out maneuvering and feed to PID controller

    prevTime = currTime
  }

  override fun isFinished(): Boolean {
    return timer.hasElapsed(trajectory.totalTimeSeconds)
  }

  override fun end(interrupted: Boolean) {
    drivetrain.stop()
  }
}
