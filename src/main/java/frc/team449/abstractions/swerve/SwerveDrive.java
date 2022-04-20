package frc.team449.abstractions.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.abstractions.DriveSubsystem;
import frc.team449.system.AHRS;
import frc.team449.system.motor.WrappedMotor;
import java.util.Arrays;
import java.util.function.Supplier;
import org.jetbrains.annotations.NotNull;

public class SwerveDrive extends SubsystemBase implements DriveSubsystem {

  private final SwerveModule[] modules;
  private final AHRS ahrs;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  public final double maxLinearSpeed;
  public final double maxRotSpeed;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

  public SwerveDrive(
      AHRS ahrs, double maxLinearSpeed, double maxRotSpeed, SwerveModule... modules) {
    this.modules = modules;
    this.ahrs = ahrs;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxRotSpeed = maxRotSpeed;
    this.kinematics =
        new SwerveDriveKinematics(
            Arrays.stream(this.modules)
                .map(module -> module.location)
                .toArray(Translation2d[]::new));
    this.odometry = new SwerveDriveOdometry(this.kinematics, ahrs.getHeading());
  }

  /**
   * Create a square swerve drivetrain
   *
   * @param ahrs Gyro used for robot heading
   * @param maxLinearSpeed Max speed (m/s) at which the robot can translate
   * @param maxRotSpeed Max speed (rad/s) at which the robot can turn in place
   * @param frontLeftDriveMotor
   * @param frontRightDriveMotor
   * @param backLeftDriveMotor
   * @param backRightDriveMotor
   * @param frontLeftTurnMotor
   * @param frontRightTurnMotor
   * @param backLeftTurnMotor
   * @param backRightTurnMotor
   * @param frontLeftLocation Location of the front left module
   * @param driveMotorController Supplier to make copies of the same driving PID controllers for all
   *     the modules
   * @param turnMotorController Supplier to make copies of the same turning PID controllers for all
   *     the modules
   * @param driveFeedforward Driving feedforward for all the modules
   * @param turnFeedforward Turning feedforward for all the modules
   * @return
   */
  public static SwerveDrive squareDrive(
      AHRS ahrs,
      double maxLinearSpeed,
      double maxRotSpeed,
      @NotNull WrappedMotor frontLeftDriveMotor,
      @NotNull WrappedMotor frontRightDriveMotor,
      @NotNull WrappedMotor backLeftDriveMotor,
      @NotNull WrappedMotor backRightDriveMotor,
      @NotNull WrappedMotor frontLeftTurnMotor,
      @NotNull WrappedMotor frontRightTurnMotor,
      @NotNull WrappedMotor backLeftTurnMotor,
      @NotNull WrappedMotor backRightTurnMotor,
      @NotNull Translation2d frontLeftLocation,
      @NotNull Supplier<PIDController> driveMotorController,
      @NotNull Supplier<ProfiledPIDController> turnMotorController,
      @NotNull SimpleMotorFeedforward driveFeedforward,
      @NotNull SimpleMotorFeedforward turnFeedforward) {
    // todo implement
    return null;
  }

  /**
   * @param xVel The robot's desired x velocity (m/s)
   * @param yVel The robot's desired y velocity (m/s)
   * @param rotVel The robot's desired rotational velocity (rad/s)
   * @param fieldRelative true if x and y are relative to the field, false if relative to the robot
   */
  public void set(double xVel, double yVel, double rotVel, boolean fieldRelative) {
    if (fieldRelative) {
      this.set(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, this.ahrs.getHeading()));
    } else {
      this.set(new ChassisSpeeds(xVel, yVel, rotVel));
    }
  }

  @Override
  public void set(ChassisSpeeds desiredSpeeds) {
    this.desiredSpeeds = desiredSpeeds;
  }

  @Override
  public Rotation2d getHeading() {
    return this.ahrs.getHeading();
  }

  @Override
  public void setPose(@NotNull Pose2d pose) {
    this.odometry.resetPosition(pose, ahrs.getHeading());
  }

  @Override
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  @Override
  public void stop() {
    this.set(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public void periodic() {
    this.odometry.update(
        ahrs.getHeading(),
        Arrays.stream(this.modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new));

    var desiredModuleStates = this.kinematics.toSwerveModuleStates(this.desiredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, this.maxLinearSpeed);

    for (int i = 0; i < this.modules.length; i++) {
      this.modules[i].set(desiredModuleStates[i]);
    }
  }
}