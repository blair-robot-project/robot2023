package frc.team449.control.differential;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class OIArcade {
  /**
   * @param drive The drivetrain
   * @param fwdBackThrottle Throttle to get forward-backward movement
   * @param rotThrottle Throttle to get rotation
   * @param fwdBackRamp Used for limiting forward-backward acceleration
   * @param rotRamp Used for limiting rotational acceleration
   */
  public static Supplier<ChassisSpeeds> create(
      DifferentialDrive drive,
      DoubleSupplier fwdBackThrottle,
      DoubleSupplier rotThrottle,
      SlewRateLimiter fwdBackRamp,
      SlewRateLimiter rotRamp) {
    return () -> {
      var fwdBackScaled = fwdBackThrottle.getAsDouble() * drive.maxLinearSpeed;
      var rotScaled = rotThrottle.getAsDouble() * drive.maxRotSpeed;
      return new ChassisSpeeds(
          fwdBackRamp.calculate(fwdBackScaled), 0, rotRamp.calculate(rotScaled));
    };
  }
}
