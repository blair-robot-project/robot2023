package frc.team449.control.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.control.DriveSubsystem;
import frc.team449.control.differential.DifferentialDrive;
import frc.team449.control.holonomic.SwerveDrive;

import java.util.function.BiFunction;
import org.jetbrains.annotations.NotNull;

public class AutoDriveCommand<T extends DriveSubsystem> extends CommandBase {
  private final T drivetrain;
  private final Trajectory trajectory;
  private final BiFunction<Pose2d, Trajectory.State, ChassisSpeeds> controller;
  private final boolean resetPose;
  private double startTime;

  /**
   * @param drivetrain Drivetrain to execute command on
   * @param trajectory Trajectory to follow
   * @param controller A controller that outputs the next ChassisSpeeds given the current pose and
   *     the next desired State
   * @param resetPose Whether to reset pose to initial pose of trajectory when initialized
   */
  public AutoDriveCommand(
      @NotNull T drivetrain,
      @NotNull Trajectory trajectory,
      @NotNull BiFunction<Pose2d, Trajectory.State, ChassisSpeeds> controller,
      boolean resetPose) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    this.controller = controller;
    this.resetPose = resetPose;
  }

  public static AutoDriveCommand<SwerveDrive> swerveDriveCommand(
      SwerveDrive drivetrain,
      @NotNull Trajectory trajectory,
      @NotNull HolonomicDriveController controller,
      double startHeading,
      double endHeading,
      boolean resetPose) {
    var totalTime = trajectory.getTotalTimeSeconds();
    return new AutoDriveCommand<>(
        drivetrain,
        trajectory,
        (currentPose, desiredState) ->
            controller.calculate(
                currentPose,
                desiredState,
                Rotation2d.fromDegrees(
                    MathUtil.interpolate(
                        startHeading, endHeading, desiredState.timeSeconds / totalTime))),
        resetPose);
  }

  public static AutoDriveCommand<DifferentialDrive> tankDriveCommand(
      DifferentialDrive drivetrain,
      @NotNull Trajectory trajectory,
      double startHeading,
      double endHeading,
      boolean resetPose) {
    var controller = new RamseteController();
    return new AutoDriveCommand<>(drivetrain, trajectory, controller::calculate, resetPose);
  }

  /** The time required for the trajectory to complete */
  public double getTotalTime() {
    return trajectory.getTotalTimeSeconds();
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    if (this.resetPose) {
      drivetrain.setPose(trajectory.getInitialPose());
    }
  }

  @Override
  public void execute() {
    drivetrain.set(
        this.controller.apply(drivetrain.getPose(), trajectory.sample(Timer.getFPGATimestamp())));
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Shuffleboard.addEventMarker(
          "AutoDriveCommand interrupted! Stopping the robot.",
          this.getClass().getSimpleName(),
          EventImportance.kNormal);
    }
    drivetrain.stop();
    Shuffleboard.addEventMarker(
        "AutoDriveCommand end.", this.getClass().getSimpleName(), EventImportance.kNormal);
  }
}
