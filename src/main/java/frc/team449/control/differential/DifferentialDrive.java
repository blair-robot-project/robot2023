package frc.team449.control.differential;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.team449.control.DriveSubsystem;
import frc.team449.system.AHRS;
import frc.team449.system.motor.WrappedMotor;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/** A differential drive with closed-loop velocity control using PID */
public class DifferentialDrive implements DriveSubsystem {

  /** The left side of the tank drive */
  private final WrappedMotor leftLeader;
  /** The right side of the tank drive */
  private final WrappedMotor rightLeader;
  /** The gyro used for the robot */
  private final AHRS ahrs;
  /**
   * The kinematics used to convert {@link DifferentialDriveWheelSpeeds} to {@link ChassisSpeeds}
   */
  public final DifferentialDriveKinematics kinematics;
  /** Odometry to keep track of where the robot is */
  private final DifferentialDriveOdometry odometry;
  /** Feedforward calculator */
  private final SimpleMotorFeedforward feedforward;

  /** Velocity PID controller for left side */
  @Config.PIDController
  private final PIDController leftPID;

  /** Velocity PID controller for right side */
  private final PIDController rightPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;
  /** Max speed going straight (m/s) */
  public final double maxLinearSpeed;
  /** Max speed when rotating (rad/s) */
  public final double maxRotSpeed;

  @Log.ToString
  private DifferentialDriveWheelSpeeds desiredSpeeds;

  public DifferentialDrive(
    WrappedMotor leftLeader,
    WrappedMotor rightLeader,
    AHRS ahrs,
    SimpleMotorFeedforward feedforward,
    PIDController velPID,
    double trackWidth,
    double maxLinearSpeed
  ) {
    this.leftLeader = leftLeader;
    this.rightLeader = rightLeader;
    this.ahrs = ahrs;
    this.leftPID =
      new PIDController(velPID.getP(), velPID.getI(), velPID.getD());
    this.rightPID =
      new PIDController(velPID.getP(), velPID.getI(), velPID.getD());
    this.feedforward = feedforward;
    this.trackWidth = trackWidth;
    this.maxLinearSpeed = maxLinearSpeed;
    // todo check this!
    this.maxRotSpeed = 2 * maxLinearSpeed / trackWidth;
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

  @Log
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

    desiredSpeeds.desaturate(this.maxLinearSpeed);
    var leftVel = desiredSpeeds.leftMetersPerSecond;
    var rightVel = desiredSpeeds.rightMetersPerSecond;
    leftLeader.setVoltage(
      feedforward.calculate(leftVel) +
      leftPID.calculate(leftLeader.getVelocity(), leftVel)
    );
    rightLeader.setVoltage(
      feedforward.calculate(rightVel) +
      rightPID.calculate(rightLeader.getVelocity(), rightVel)
    );
  }
}
