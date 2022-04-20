package frc.team449.abstractions.tank;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.system.AHRS;
import frc.team449.system.motor.WrappedMotor;

/**
 * A tank drive with closed-loop velocity control using PID
 */
public class TankDrive implements DriveSubsystem {
  /** The left side of the tank drive */
  private final WrappedMotor leftLeader;
  /** The right side of the tank drive */
  private final WrappedMotor rightLeader;
  /** The gyro used for the robot */
  private final AHRS ahrs;
  /**
   * The kinematics used to convert {@link DifferentialDriveWheelSpeeds} to {@link ChassisSpeeds}
   */
  private final DifferentialDriveKinematics kinematics;
  /** odometer that keeps track of where the robot is */
  private final DifferentialDriveOdometry odometry;
  /** Feedforward calculator */
  private final SimpleMotorFeedforward feedforward;
  /** Velocity PID controller for left side */
  private final PIDController leftPID;
  /** Velocity PID controller for right side */
  private final PIDController rightPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;

  private DifferentialDriveWheelSpeeds desiredSpeeds;

  public TankDrive(
      WrappedMotor leftLeader,
      WrappedMotor rightLeader,
      AHRS ahrs,
      SimpleMotorFeedforward feedforward,
      PIDController velPID,
      double trackWidth) {
    this.leftLeader = leftLeader;
    this.rightLeader = rightLeader;
    this.ahrs = ahrs;
    this.leftPID = new PIDController(velPID.getP(), velPID.getI(), velPID.getD());
    this.rightPID = new PIDController(velPID.getP(), velPID.getI(), velPID.getD());
    this.feedforward = feedforward;
    this.trackWidth = trackWidth;
    this.kinematics = new DifferentialDriveKinematics(trackWidth);
    this.odometry = new DifferentialDriveOdometry(ahrs.getHeading());
  }

  /**
   * Convert from x, y, rotation to left and right speeds
   *
   * @param desiredSpeeds The {@link ChassisSpeeds} desired for the drive
   */
  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    this.desiredSpeeds = kinematics.toWheelSpeeds(desiredSpeeds);
  }

  @Override
  public Rotation2d getHeading() {
    return ahrs.getHeading();
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  @Override
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /** Reset odometry tracker to current robot pose */
  @Override
  public void setPose(Pose2d pose) {
    leftLeader.encoder.resetPosition(0);
    rightLeader.encoder.resetPosition(0);
    ahrs.setHeading(pose.getRotation());
    this.odometry.resetPosition(pose, ahrs.getHeading());
  }

  @Override
  public void stop() {
    this.desiredSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
  }

  /** Periodically update the odometry */
  @Override
  public void periodic() {
    var leftPosition = this.leftLeader.getPosition();
    var rightPosition = this.rightLeader.getPosition();
    this.odometry.update(this.getHeading(), leftPosition, rightPosition);

    var leftVel = desiredSpeeds.leftMetersPerSecond;
    var rightVel = desiredSpeeds.rightMetersPerSecond;
    leftLeader.setVoltage(
        feedforward.calculate(leftVel) + leftPID.calculate(leftLeader.getVelocity(), leftVel));
    rightLeader.setVoltage(
        feedforward.calculate(rightVel) + rightPID.calculate(rightLeader.getVelocity(), rightVel));
  }
}
