package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.EventImportance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem
import frc.team449.control.differential.DifferentialDrive
import frc.team449.control.holonomic.HolonomicDrive

/**
 * @param drivetrain Drivetrain to execute command on
 * @param trajectory Trajectory to follow
 * @param controller A controller that outputs the next ChassisSpeeds given the current pose and
 * the next desired State
 * @param resetPose Whether to reset pose to initial pose of trajectory when initialized
 */
class AutoDriveCommand<T : DriveSubsystem>(
  val drivetrain: T,
  val trajectory: Trajectory,
  val controller: (Pose2d, Trajectory.State) -> ChassisSpeeds,
  val resetPose: Boolean
) : CommandBase() {
  private var startTime = 0.0

  init {
    addRequirements(drivetrain)
  }

  /** The time required for the trajectory to complete  */
  val totalTime: Double
    get() = trajectory.totalTimeSeconds

  override fun initialize() {
    startTime = Timer.getFPGATimestamp()
    if (resetPose) {
      drivetrain.pose = trajectory.initialPose
    }
  }

  override fun execute() {
    drivetrain.set(
      controller(drivetrain.pose, trajectory.sample(Timer.getFPGATimestamp() - startTime))
    )
  }

  override fun isFinished(): Boolean {
    return Timer.getFPGATimestamp() - startTime >= trajectory.totalTimeSeconds
  }

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
        "AutoDriveCommand interrupted! Stopping the robot.",
        this.javaClass.simpleName,
        EventImportance.kNormal
      )
    }
    drivetrain.stop()
    Shuffleboard.addEventMarker(
      "AutoDriveCommand end.", this.javaClass.simpleName, EventImportance.kNormal
    )
  }

  companion object {
    /**
     * @return command to automatically drive a holonomic drive (eg. swerve, mecanum, ...) to a desired destination
     */
    fun holonomicDriveCommand(
      drivetrain: HolonomicDrive,
      trajectory: PathPlannerTrajectory,
      controller: HolonomicDriveController,
      resetPose: Boolean
    ): AutoDriveCommand<HolonomicDrive> {
      val totalTime = trajectory.totalTimeSeconds
      val startHeading = trajectory.initialState.holonomicRotation.degrees
      val endHeading = trajectory.endState.holonomicRotation.degrees
      return AutoDriveCommand(
        drivetrain,
        trajectory,
        { currentPose, desiredState ->
          controller.calculate(
            currentPose,
            desiredState,
            Rotation2d.fromDegrees(
              MathUtil.interpolate(
                startHeading, endHeading, desiredState.timeSeconds / totalTime
              )
            )
          )
        },
        resetPose
      )
    }

    fun differentialDriveCommand(
      drivetrain: DifferentialDrive,
      trajectory: Trajectory,
      resetPose: Boolean
    ): AutoDriveCommand<DifferentialDrive> {
      val controller = RamseteController()
      return AutoDriveCommand(
        drivetrain,
        trajectory,
        { currentPose, desiredState ->
          controller.calculate(currentPose, desiredState)
        },
        resetPose
      )
    }
  }
}
