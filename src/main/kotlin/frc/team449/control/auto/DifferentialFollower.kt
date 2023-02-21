package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.differential.DifferentialDrive
import frc.team449.robot2023.constants.RobotConstants

/**
 * @param drivetrain Holonomic Drivetrain used
 * @param trajectory Path Planner trajectory to follow
 * @param resetPose Whether to reset the robot pose to the first pose in the trajectory
 * @param translationTol Tolerance between the translation of the actual and desired end pose of the trajectory in meters
 * @param angleTol Tolerance between the angle of the actual and desired end pose of the trajectory in radians
 * @param timeout Maximum time to wait for the robot to fix its end pose. Gives an upper bound of the time the trajectory takes
 */
class DifferentialFollower(
  private val drivetrain: DifferentialDrive,
  private val trajectory: PathPlannerTrajectory,
  private val resetPose: Boolean,
  private val translationTol: Double = 0.1,
  private val angleTol: Double = 0.5,
  private val timeout: Double = 5.0
) : CommandBase() {

  private val timer = Timer()

  private val controller = RamseteController()

  // MAKE SURE YOUR ORIGINAL PATH IS FOR THE BLUE ALLIANCE
  private val transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
    trajectory,
    RobotConstants.ALLIANCE_COLOR
  )

  override fun initialize() {
    if (resetPose) {
      // reset the position of the robot to where we start this path(trajectory)
      drivetrain.pose = trajectory.initialState.poseMeters
    }

    controller.setTolerance(Pose2d(translationTol, translationTol, Rotation2d(angleTol)))

    PathPlannerServer.sendActivePath(transformedTrajectory.states)

    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()

    val reference = transformedTrajectory.sample(currTime) as PathPlannerTrajectory.PathPlannerState

    val currentPose = drivetrain.pose

    PathPlannerServer.sendPathFollowingData(
      Pose2d(
        reference.poseMeters.translation,
        reference.holonomicRotation
      ),
      currentPose
    )

    drivetrain.set(
      controller.calculate(
        currentPose,
        reference
      )
    )
  }

  override fun isFinished(): Boolean {
    return timer.hasElapsed(trajectory.totalTimeSeconds) && controller.atReference() ||
      (timer.hasElapsed(trajectory.totalTimeSeconds + timeout))
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }
}
