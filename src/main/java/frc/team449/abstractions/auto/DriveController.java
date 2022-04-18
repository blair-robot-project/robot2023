package frc.team449.abstractions.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * A controller for making a drivetrain reach a desired State
 * (wrapper around HolonomicDriveController or RamseteController)
 */
@FunctionalInterface
public interface DriveController {
  /**
   * Calculate the next speeds to give the drivetrain given its current pose,
   * desired state, and number of seconds elapsed since the start of the
   * trajectory
   */
  ChassisSpeeds calculate(
      Pose2d currentPose,
      Trajectory.State desiredState,
      double time);
}
