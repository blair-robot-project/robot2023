package frc.team449.abstractions.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class OISwerve implements Supplier<ChassisSpeeds> {

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;
  private final SlewRateLimiter xRamp;
  private final SlewRateLimiter yRamp;
  private final SlewRateLimiter rotRamp;

  public OISwerve(
      SlewRateLimiter xRamp,
      SlewRateLimiter yRamp,
      SlewRateLimiter rotRamp,
      DoubleSupplier xThrottle,
      DoubleSupplier yThrottle,
      DoubleSupplier rotThrottle) {
    this.xRamp = xRamp;
    this.yRamp = yRamp;
    this.rotRamp = rotRamp;
    this.rotThrottle = rotThrottle;
    this.yThrottle = yThrottle;
    this.xThrottle = xThrottle;
  }

  /** @return the {@link ChassisSpeeds} for the given x, y and rotation input from the joystick */
  @Override
  public ChassisSpeeds get() {
    var x = xRamp.calculate(xThrottle.getAsDouble());
    var y = yRamp.calculate(yThrottle.getAsDouble());
    var rot = rotRamp.calculate(rotThrottle.getAsDouble());
    return new ChassisSpeeds(x, y, rot);
  }
}
