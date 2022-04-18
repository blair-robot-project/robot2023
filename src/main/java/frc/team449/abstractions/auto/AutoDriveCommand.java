package frc.team449.abstractions.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.abstractions.SwerveDrive;
import frc.team449.abstractions.TankDrive;
import org.jetbrains.annotations.NotNull;

public class AutoDriveCommand<T extends DriveSubsystem> extends CommandBase {
  private final T drivetrain;
  private final Trajectory trajectory;
  private final DriveController controller;
  private final boolean resetPose;
  private double startTime;

  /**
   * @param drivetrain Drivetrain to execute command on
   * @param trajectory Trajectory to follow
   * @param resetPose Whether to reset odometry to initial pose of trajectory when initialized
   */
  public AutoDriveCommand(
      @NotNull T drivetrain,
      @NotNull Trajectory trajectory,
      @NotNull DriveController controller,
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
        (currentPose, desiredState, time) ->
            controller.calculate(
                currentPose,
                desiredState,
                Rotation2d.fromDegrees(
                    MathUtil.interpolate(startHeading, endHeading, time / totalTime))),
        resetPose);
  }

  public static AutoDriveCommand<TankDrive> tankDriveCommand(
      TankDrive drivetrain,
      @NotNull Trajectory trajectory,
      @NotNull RamseteController controller,
      double startHeading,
      double endHeading,
      boolean resetPose) {
    return new AutoDriveCommand<>(
        drivetrain,
        trajectory,
        (currentPose, desiredState, time) -> controller.calculate(currentPose, desiredState),
        resetPose);
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
    var currTime = Timer.getFPGATimestamp();
    drivetrain.set(
        this.controller.calculate(drivetrain.getPose(), trajectory.sample(currTime), currTime));
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
