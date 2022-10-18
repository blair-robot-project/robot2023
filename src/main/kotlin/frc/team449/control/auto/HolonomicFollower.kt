package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants
import kotlin.math.PI
import kotlin.math.abs

/**
 * @param drivetrain Holonomic Drivetrain used
 * @param trajectory Path Planner trajectory to follow
 * @param resetPose Whether to reset the robot pose to the first pose in the trajectory
 * @param maxRotAcc Max rotational acceleration
 * @param maxRotVel Max rotational velocity
 * @param translationTol Tolerance between the translation of the actual and desired end pose of the trajectory in meters
 * @param angleTol Tolerance between the angle of the actual and desired end pose of the trajectory in radians
 * @param timeout Maximum time to wait for the robot to fix its end pose. Gives an upper bound of the time the trajectory takes
 */
class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: PathPlannerTrajectory,
  private val resetPose: Boolean,
  maxRotVel: Double,
  maxRotAcc: Double,
  private val translationTol: Double = 0.05,
  private val angleTol: Double = 0.05,
  private val timeout: Double = 0.05
) : CommandBase() {

  private val timer = Timer()
  private var prevTime = 0.0

  private var xController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var yController = PIDController(AutoConstants.TRANSLATION_KP, .0, .0)
  private var thetaController = ProfiledPIDController(
    AutoConstants.ROTATION_KP, .0, .0,
    TrapezoidProfile.Constraints(maxRotVel, maxRotAcc)
  )
  private val controller = HolonomicDriveController(
    xController, yController, thetaController
  )

  init {
    // controlling heading which is circular (0, 2PI)
    thetaController.enableContinuousInput(.0, 2 * PI)
  }

  override fun initialize() {
    if (resetPose) {
      // reset the position of the robot to where we start this path(trajectory)
      val start = trajectory.initialState as PathPlannerTrajectory.PathPlannerState
      drivetrain.pose = Pose2d(start.poseMeters.x, start.poseMeters.y, start.holonomicRotation)
    }

    xController.reset()
    yController.reset()
    thetaController.reset(drivetrain.pose.rotation.radians)

    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()
    val reference = trajectory.sample(currTime) as PathPlannerTrajectory.PathPlannerState
    val currentPose = drivetrain.pose

    drivetrain.set(
      controller.calculate(
        currentPose,
        reference.poseMeters,
        reference.velocityMetersPerSecond,
        reference.holonomicRotation
      )
    )

    prevTime = currTime
  }

  override fun isFinished(): Boolean {
    return (timer.hasElapsed(trajectory.totalTimeSeconds) && inTolerance()) ||
      (timer.hasElapsed(trajectory.totalTimeSeconds + timeout))
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }

  /** Check if the x y and theta of the robot are close enough to the END goal */
  private fun inTolerance(): Boolean {
    val endError = trajectory.endState.poseMeters.relativeTo(drivetrain.pose)
    return abs(endError.x) < translationTol && abs(endError.y) < translationTol && abs(endError.rotation.radians) < angleTol
  }
}
