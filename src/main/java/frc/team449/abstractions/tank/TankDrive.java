package frc.team449.abstractions.tank;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.system.AHRS;
import frc.team449.system.motor.WrappedMotor;

public class TankDrive implements DriveSubsystem {
  /** The left side of the tank drive */
  private final WrappedMotor leftLeader;
  /** The right side of the tank drive */
  private final WrappedMotor rightLeader;
  /** The gyro used for the robot */
  private final AHRS ahrs;
  /**
   * the kinematics used to convert {@link DifferentialDriveWheelSpeeds} to {@link ChassisSpeeds}
   */
  private final DifferentialDriveKinematics driveKinematics;
  /** odometer that keeps track of where the robot is */
  private final DifferentialDriveOdometry odometry;
  /** Feedforward calculator */
  public final SimpleMotorFeedforward feedforward;
  /** Velocity PID controller for both sides */
  public final PIDController velPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;

  public TankDrive(WrappedMotor leftLeader,
                   WrappedMotor rightLeader,
                   SimpleMotorFeedforward feedforward,
                   PIDController velPID,
                   AHRS ahrs,
                   double trackWidth) { // TODO
    this.trackWidth = trackWidth;
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    this.ahrs = ahrs;
    this.velPID = velPID;
    this.feedforward = feedforward;
    this.leftLeader = leftLeader;
    this.rightLeader = rightLeader;
    odometry = new DifferentialDriveOdometry(ahrs.getHeading());
  }

  /**
   * convert from x, y, rotation to left and right speeds
   * apply feedforward to each desired speed, then set the voltage
   * @param desiredSpeeds the {@link ChassisSpeeds} desired for the drive
   */
  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    var tankSpeeds = driveKinematics.toWheelSpeeds(desiredSpeeds);
    var leftVoltage = feedforward.calculate(tankSpeeds.leftMetersPerSecond);
    var rightVoltage = feedforward.calculate(tankSpeeds.rightMetersPerSecond);
    leftLeader.setVoltage(leftVoltage);
    rightLeader.setVoltage(rightVoltage);
  }

  @Override
  public AHRS getAHRS() {
    return this.ahrs;
  }

  public PIDController getVelPID() {
    return velPID;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
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
    resetEncoders();
    ahrs.setHeading(pose.getRotation());
    this.odometry.resetPosition(pose, ahrs.getHeading());
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    leftLeader.set(0);
    rightLeader.set(0);
  }

  /** Reset the position of the drive if it has encoders. */

  public void resetEncoders(){
    leftLeader.encoder.resetPosition(0);
    rightLeader.encoder.resetPosition(0);
  }
  /**
   * Periodically update the odometry
   */
  @Override
  public void periodic(){
    var leftPosition = this.leftLeader.getPosition();
    var rightPosition = this.rightLeader.getPosition();
    this.odometry.update(this.ahrs.getHeading(), leftPosition, rightPosition);
  }
}
