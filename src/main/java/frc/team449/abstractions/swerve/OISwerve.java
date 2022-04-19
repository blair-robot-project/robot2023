package frc.team449.abstractions.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class OISwerve implements Supplier<ChassisSpeeds> {

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;

  private final SlewRateLimiter xFilter;
  private final SlewRateLimiter yFilter;
  private final SlewRateLimiter rotFilter;

  @Override
  public ChassisSpeeds get() {
    var x = xThrottle.getAsDouble();
    var y = yThrottle.getAsDouble();
    var rot = rotThrottle.getAsDouble();

    var mag = Math.hypot(x, y);
  }
}
