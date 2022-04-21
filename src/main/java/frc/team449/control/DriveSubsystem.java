package frc.team449.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.oblarg.oblog.Loggable;

/** A drivetrain that uses closed-loop velocity control. */
public interface DriveSubsystem extends Subsystem, Loggable {
  /** Set the desired speeds to go at. */
  void set(ChassisSpeeds desiredSpeeds);

  Rotation2d getHeading();

  /** Reset the drivetrain's pose. Should really only be done at the start of auto. */
  void setPose(Pose2d pose);

  Pose2d getPose();

  /** Set both motors' voltage to 0. */
  void stop();
}
