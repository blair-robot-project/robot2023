package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.differential.DifferentialDrive

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
  private var prevTime = 0.0

  private val controller = RamseteController()

  override fun initialize() {
    controller.setTolerance(Pose2d(Translation2d(translationTol, translationTol), Rotation2d(angleTol)))
    if (resetPose) {
      // reset the position of the robot to where we start this path(trajectory)
      drivetrain.pose = trajectory.initialState.poseMeters
    }
    controller.setTolerance(Pose2d(translationTol, translationTol, Rotation2d(angleTol)))

    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()

    val reference = PathPlannerTrajectory.transformStateForAlliance(
      trajectory.sample(currTime) as PathPlannerTrajectory.PathPlannerState,
      DriverStation.getAlliance()
    )

    val currentPose = drivetrain.pose

    drivetrain.set(
      controller.calculate(
        currentPose,
        reference
      )
    )

    prevTime = currTime
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
