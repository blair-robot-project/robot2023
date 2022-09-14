package frc.team449.control.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants
import kotlin.math.PI

class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: SwerveTrajectory,
  private val resetPose: Boolean
) : CommandBase() {
  private val timer = Timer()
  private var prevTime = 0.0

  private var xController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var yController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var thetaController = PIDController(AutoConstants.ROTATION_KP, .0, .0)

  override fun initialize() {
    thetaController.enableContinuousInput(0.0, PI * 2)
    timer.start()
    if (resetPose) {
      drivetrain.pose = trajectory.getInitialPose()
    }
  }

  override fun execute() {
    val currTime = timer.get()
    val dt = currTime - prevTime
    val reference = trajectory.sample(currTime)
    val currentPose = drivetrain.pose

    val vx = xController.calculate(currentPose.x, reference.pose.x)
    val vy = yController.calculate(currentPose.y, reference.pose.y)
    val omega = -thetaController.calculate(currentPose.rotation.radians, dt) - reference.velocity.z

    drivetrain.set(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.heading))

    prevTime = currTime
  }

  override fun isFinished(): Boolean {
    return timer.hasElapsed(trajectory.getTotalTime())
  }

  override fun end(interrupted: Boolean) {
    drivetrain.stop()
  }
}
