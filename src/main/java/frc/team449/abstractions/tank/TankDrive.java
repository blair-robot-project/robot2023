package frc.team449.abstractions.tank;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.system.AHRS;
import frc.team449.system.motor.WrappedMotor;
import org.jetbrains.annotations.NotNull;

public class TankDrive implements DriveSubsystem {
  /** The left side of the tank drive */
  private final WrappedMotor leftLeader;
  /** The right side of the tank drive */
  private final WrappedMotor rightLeader;
  /** The gyro used for the robot */
  private final AHRS ahrs;
  /**
   * the kinematics used to convert {@link ChassisSpeeds} to
   * {@link DifferentialDriveWheelSpeeds}
   */
  private final DifferentialDriveKinematics kinematics;
  /** Odometry to keeps track of where the robot is */
  private final DifferentialDriveOdometry odometry;
  /** Feedforward calculator for both sides */
  public final SimpleMotorFeedforward feedforward;
  /** Velocity PID controller for left side */
  public final PIDController leftPID;
  /** Velocity PID controller for right side */
  public final PIDController rightPID;

  public TankDrive(
      @NotNull WrappedMotor leftLeader,
      @NotNull WrappedMotor rightLeader,
      @NotNull AHRS ahrs,
      @NotNull SimpleMotorFeedforward feedforward,
      @NotNull PIDController leftPID,
      @NotNull PIDController rightPID,
      double trackwidth) {
    this.leftLeader = leftLeader;
    this.rightLeader = rightLeader;
    this.ahrs = ahrs;
    this.feedforward = feedforward;
    this.leftPID = leftPID;
    this.rightPID = rightPID;
    this.kinematics = new DifferentialDriveKinematics(trackwidth);
    this.odometry = new DifferentialDriveOdometry(ahrs.getHeading());
  }

  /**
   * Set the desired speeds for the left and right side
   * @see TankDrive#set(ChassisSpeeds)
   */
  public void set(double leftVel, double rightVel) {

  }

  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    // todo convert to DifferentialDriveSpeeds using DifferentialDriveKinematics
    // then call set(leftVel, rightVel)
  }

  @Override
  public Rotation2d getHeading() {
    // todo implement
    return null;
  }

  @Override
  public void setPose(Pose2d pose) {
    // TODO Auto-generated method stub
  }

  @Override
  public Pose2d getPose() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void periodic() {
    // todo update odometry

    // todo use pid and ff for vel control
  }
}
