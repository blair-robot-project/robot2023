package frc.team449.robot2023.commands.driveAlign

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive

/**
 * @param drive The holonomic drive you want to align with
 * @param targetPose The pose you want to drive up to
 * @param xPID The profiled PID controller with constraints you want to use for fixing X error
 * @param yPID The profiled PID controller with constraints you want to use for fixing Y error
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 * @param tolerance The allowed tolerance from the targetPose
 */
class ProfiledPoseAlign(
  private val drive: HolonomicDrive,
  private val targetPose: Pose2d,
  private val xPID: ProfiledPIDController,
  private val yPID: ProfiledPIDController,
  private val headingPID: PIDController,
  tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05))
) : CommandBase() {
  init {
    addRequirements(drive)
    headingPID.enableContinuousInput(-Math.PI, Math.PI)

    // Set tolerances from the given pose tolerance
    xPID.setTolerance(tolerance.x)
    yPID.setTolerance(tolerance.y)
    headingPID.setTolerance(tolerance.rotation.radians)

    // Set the goals for all the PID controller to the target pose
    xPID.setGoal(targetPose.x)
    yPID.setGoal(targetPose.y)
    headingPID.setpoint = targetPose.rotation.radians
  }

  override fun execute() {
    // Calculate the feedback for X, Y, and theta using their respective controllers
    val xFeedback = xPID.calculate(drive.pose.x)
    val yFeedback = yPID.calculate(drive.pose.y)
    val headingFeedback = headingPID.calculate(drive.heading.radians)

    drive.set(ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, headingFeedback, drive.heading))
  }

  override fun isFinished(): Boolean {
    return xPID.atGoal() && yPID.atGoal() && headingPID.atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
