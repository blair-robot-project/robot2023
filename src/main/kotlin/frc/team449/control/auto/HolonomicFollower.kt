package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.HolonomicDrive
import kotlin.math.PI

/**
 * @param drivetrain Holonomic Drivetrain used
 * @param path Path Planner trajectory to follow
 */
class HolonomicFollower(
  private val drivetrain: HolonomicDrive,
  private val path: HolonomicPath
) : CommandBase() {

  private val timer = Timer()
  private var prevTime = 0.0

  private val trajectory = path.trajectory
  private val xController = path.xController
  private val yController = path.yController
  private val thetaController = path.rotController
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
        Translation2d(path.translationTol, path.translationTol),
        Rotation2d(path.angleTol)
      )
    )
  }

  override fun initialize() {
    if (path.resetPose) {
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
      timer.hasElapsed(trajectory.totalTimeSeconds + path.timeout)
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }
}
