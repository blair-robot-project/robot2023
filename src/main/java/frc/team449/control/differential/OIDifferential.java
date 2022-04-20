package frc.team449.control.differential;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class OIDifferential implements Supplier<ChassisSpeeds> {

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier rotThrottle;
  private final double maxSpeedMeters;

  // put a limit on the acceleration, used to prevent tipping if robot is top-heavy
  private final SlewRateLimiter xRamp;
  private final SlewRateLimiter rotRamp;

  // whether the drive should be curvature or arcade
  private final boolean curvatureDrive;
  // if turning in place should be allowed when using curvature drive
  private final boolean turnInPlace;

  /**
   * the kinematics of the drive used to convert from {@link DifferentialDriveWheelSpeeds} to {@link
   * ChassisSpeeds}
   */
  private final DifferentialDriveKinematics kinematics;

  public OIDifferential(
      DoubleSupplier xThrottle,
      DoubleSupplier rotThrottle,
      SlewRateLimiter xRamp,
      SlewRateLimiter rotRamp,
      boolean curvatureDrive,
      boolean turnInPlace,
      double trackWidth,
      double maxSpeedMeters) {
    this.xThrottle = xThrottle;
    this.rotThrottle = rotThrottle;
    this.xRamp = xRamp;
    this.rotRamp = rotRamp;
    this.curvatureDrive = curvatureDrive;
    this.turnInPlace = turnInPlace;
    kinematics = new DifferentialDriveKinematics(trackWidth);
    this.maxSpeedMeters = maxSpeedMeters;
  }

  @Override
  public ChassisSpeeds get() {
    var xSpeed = xThrottle.getAsDouble();
    var rotSpeed = rotThrottle.getAsDouble();

    // apply the limit on the acceleration
    xSpeed = xRamp.calculate(xSpeed);
    rotSpeed = rotRamp.calculate(rotSpeed);

    if (curvatureDrive) return curvatureDrive(xSpeed, rotSpeed, this.turnInPlace);

    return arcadeDrive(xSpeed, rotSpeed);
  }

  private ChassisSpeeds arcadeDrive(double xSpeed, double rotSpeed) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(rotSpeed)), xSpeed);

    if (Double.compare(xSpeed, 0.0) >= 0) {
      // First quadrant, else second quadrant
      if (Double.compare(rotSpeed, 0.0) >= 0) {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - rotSpeed;
      } else {
        leftSpeed = xSpeed + rotSpeed;
        rightSpeed = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (Double.compare(rotSpeed, 0.0) >= 0) {
        leftSpeed = xSpeed + rotSpeed;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - rotSpeed;
      }
    }
    // Convert from % output to meters/s
    leftSpeed *= maxSpeedMeters;
    rightSpeed *= maxSpeedMeters;

    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  private ChassisSpeeds curvatureDrive(double xSpeed, double rotSpeed, boolean allowTurnInPlace) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

    double leftSpeed;
    double rightSpeed;

    if (allowTurnInPlace) {
      leftSpeed = xSpeed + rotSpeed;
      rightSpeed = xSpeed - rotSpeed;
    } else {
      leftSpeed = xSpeed + Math.abs(xSpeed) * rotSpeed;
      rightSpeed = xSpeed - Math.abs(xSpeed) * rotSpeed;
    }

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }
    // Convert from % output to meters/s
    leftSpeed *= this.maxSpeedMeters;
    rightSpeed *= this.maxSpeedMeters;

    return this.kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }
}
