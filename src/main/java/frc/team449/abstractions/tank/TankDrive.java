package frc.team449.abstractions.tank;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  /** Velocity PID controller for left side */
  public final PIDController leftVelPID;
  /** Velocity PID controller for right side */
  public final PIDController rightVelPID;
  /** The track width of the robot/distance between left and right wheels in meters */
  public final double trackWidth;

  public TankDrive(WrappedMotor leftLeader, WrappedMotor rightLeader, AHRS ahrs) { // TODO
    driveKinematics = new DifferentialDriveKinematics();
  }

  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    // TODO Auto-generated method stub

  }

  @Override
  public AHRS getAHRS() {
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
}
