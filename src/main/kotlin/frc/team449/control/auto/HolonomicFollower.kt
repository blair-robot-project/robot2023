package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants
import kotlin.math.PI

/**
 * @param drivetrain Holonomic Drivetrain used
 * @param trajectory Path Planner trajectory to follow
 * @param resetPose Whether to reset the robot pose to the first pose in the trajectory
 * @param translationTol Tolerance between the translation of the actual and desired end pose of the trajectory in meters
 * @param angleTol Tolerance between the angle of the actual and desired end pose of the trajectory in radians
 * @param timeout Maximum time to wait for the robot to fix its end pose. Gives an upper bound of the time the trajectory takes
 */
class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val trajectory: PathPlannerTrajectory,
  private val resetPose: Boolean,
  private val translationTol: Double = 0.005,
  private val angleTol: Double = 0.001,
  private val timeout: Double = 5.0
) : CommandBase() {

  private val timer = Timer()
  private var prevTime = 0.0

  private var xController = PIDController(AutoConstants.X_KP, .0, .0)
  private var yController = PIDController(AutoConstants.Y_KP, .0, .0)
  private var thetaController = PIDController(AutoConstants.ROTATION_KP, .0, .0)

  private val controller = PPHolonomicDriveController(
    xController, yController, thetaController
  )

  init {
    // require the drivetrain to interrupt
    addRequirements(drivetrain)

    // controlling heading which is circular, [0, 2*PI)
    thetaController.enableContinuousInput(.0, 2 * PI)

    controller.setTolerance(
      Pose2d(
        Translation2d(translationTol, translationTol),
        Rotation2d(angleTol)
      )
    )
  }

  override fun initialize() {
    if (resetPose) {
      // initially assume the robot is at this pose already
      drivetrain.pose = trajectory.initialHolonomicPose
    }

    // reset the controllers so that the error from last run doesn't transfer
    xController.reset()
    yController.reset()
    thetaController.reset()

    // reset timer from last run and restart for this run
    timer.reset()
    timer.start()
  }

  override fun execute() {
    val currTime = timer.get()
    val reference = trajectory.sample(currTime) as PathPlannerTrajectory.PathPlannerState
    val currentPose = drivetrain.pose

    drivetrain.set(
      controller.calculate(
        currentPose,
        reference
      )
    )

    prevTime = currTime
  }

  /**
   * @return if the robot reached ending position by estimated end time
   */
  override fun isFinished(): Boolean {
    return (timer.hasElapsed(trajectory.totalTimeSeconds) && controller.atReference()) ||
      timer.hasElapsed(trajectory.totalTimeSeconds + timeout)
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }
}
