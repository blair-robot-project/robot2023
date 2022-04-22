package frc.team449.control.auto

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
import frc.team449.control.holonomic.SwerveDrive
import java.util.function.BiFunction

class AutoDriveCommand<T : DriveSubsystem?>(
  drivetrain: T,
  trajectory: Trajectory,
  controller: BiFunction<Pose2d, Trajectory.State, ChassisSpeeds>,
  resetPose: Boolean
) : CommandBase() {
  private val drivetrain: T
  private val trajectory: Trajectory
  private val controller: BiFunction<Pose2d, Trajectory.State, ChassisSpeeds>
  private val resetPose: Boolean
  private var startTime = 0.0

  /**
   * @param drivetrain Drivetrain to execute command on
   * @param trajectory Trajectory to follow
   * @param controller A controller that outputs the next ChassisSpeeds given the current pose and
   * the next desired State
   * @param resetPose Whether to reset pose to initial pose of trajectory when initialized
   */
  init {
    addRequirements(drivetrain)
    this.drivetrain = drivetrain
    this.trajectory = trajectory
    this.controller = controller
    this.resetPose = resetPose
  }

  /** The time required for the trajectory to complete  */
  val totalTime: Double
    get() = trajectory.totalTimeSeconds

  override fun initialize() {
    startTime = Timer.getFPGATimestamp()
    if (resetPose) {
      drivetrain!!.pose = trajectory.initialPose
    }
  }

  override fun execute() {
    drivetrain!!.set(
      controller.apply(drivetrain.pose, trajectory.sample(Timer.getFPGATimestamp()))
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
    drivetrain!!.stop()
    Shuffleboard.addEventMarker(
      "AutoDriveCommand end.", this.javaClass.simpleName, EventImportance.kNormal
    )
  }

  companion object {
    fun swerveDriveCommand(
      drivetrain: SwerveDrive,
      trajectory: Trajectory,
      controller: HolonomicDriveController,
      startHeading: Double,
      endHeading: Double,
      resetPose: Boolean
    ): AutoDriveCommand<SwerveDrive> {
      val totalTime = trajectory.totalTimeSeconds
      return AutoDriveCommand(
        drivetrain,
        trajectory,
        { currentPose: Pose2d?, desiredState: Trajectory.State ->
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

    fun tankDriveCommand(
      drivetrain: DifferentialDrive,
      trajectory: Trajectory,
      startHeading: Double,
      endHeading: Double,
      resetPose: Boolean
    ): AutoDriveCommand<DifferentialDrive> {
      val controller = RamseteController()
      return AutoDriveCommand(drivetrain, trajectory, { currentPose: Pose2d?, desiredState: Trajectory.State? -> controller.calculate(currentPose, desiredState) }, resetPose)
    }
  }
}
