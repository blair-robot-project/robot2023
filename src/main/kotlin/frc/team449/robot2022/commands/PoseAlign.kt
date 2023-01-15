package frc.team449.robot2022.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2022.Robot
import frc.team449.robot2022.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config

class PoseAlign(
  private val robot: Robot,
  private val targetPose: Pose2d,
  @field:Config.PIDController(name = "Pose Align X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Pose Align Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Pose Align Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val poseTolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d(0.05)),
  private val timeout: Double = 0.75,
  private val maxSpeeds: PathConstraints = PathConstraints(3.0, 1.5)
) {

  fun generateCommand(): Command {
    val startPointRotation = targetPose.translation.minus(robot.drive.pose.translation).angle
    val endPointRotation = robot.drive.pose.translation.minus(targetPose.translation).angle

    val traj = PathPlanner.generatePath(
      maxSpeeds,
      PathPoint(robot.drive.pose.translation, startPointRotation, robot.drive.pose.rotation),
      PathPoint(targetPose.translation, endPointRotation, targetPose.rotation)
    )

    val cmd = HolonomicFollower(
      robot.drive,
      traj,
      xController,
      yController,
      thetaController,
      poseTolerance,
      timeout
    )

    cmd.addRequirements(robot.drive)

    return cmd
  }
}
