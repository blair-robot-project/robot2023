package frc.team449.control.holonomic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Create an OI for controlling a holonomic drivetrain (probably swerve).
 * The x and y axes on one joystick are used to control x and y velocity (m/s),
 * while the x axis on another joystick is used to control rotational velocity (m/s).
 *
 * todo apply ramping to magnitude of difference vector, not just clamping
 */
public class OIHolonomic implements Supplier<ChassisSpeeds> {

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;
  private final SlewRateLimiter xyRamp;
  private final SlewRateLimiter rotRamp;
  private final double maxLinearSpeed;
  private final double maxRotSpeed;

  private double prevX;
  private double prevY;

  /**
   *
   * @param xThrottle
   * @param yThrottle
   * @param rotThrottle
   * @param rotRamp
   * @param maxLinearSpeed
   * @param maxRotSpeed
   * @param maxAccel Max accel, used for ramping
   */
  public OIHolonomic(
    DoubleSupplier xThrottle,
    DoubleSupplier yThrottle,
    DoubleSupplier rotThrottle,
    SlewRateLimiter xyRamp,
    SlewRateLimiter rotRamp,
    double maxLinearSpeed,
    double maxRotSpeed
  ) {
    this.rotThrottle = rotThrottle;
    this.yThrottle = yThrottle;
    this.xThrottle = xThrottle;
    this.rotRamp = rotRamp;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxRotSpeed = maxRotSpeed;
    this.xyRamp = xyRamp;
  }

  /** @return The {@link ChassisSpeeds} for the given x, y and rotation input from the joystick */
  @Override
  public ChassisSpeeds get() {
    var xScaled = xThrottle.getAsDouble() * this.maxLinearSpeed;
    var yScaled = yThrottle.getAsDouble() * this.maxLinearSpeed;

    // Ramp the translation throttles
    xyRamp.reset(0);
    var dx = this.prevX - xScaled;
    var dy = this.prevY - yScaled;
    var magDiff = Math.hypot(dx, dy);
    var magDiffRamped = xyRamp.calculate(magDiff);
    var dxClamped = dx * magDiffRamped / magDiff;
    var dyClamped = dy * magDiffRamped / magDiff;

    var xClamped = prevX + dxClamped;
    var yClamped = prevY + dyClamped;

    this.prevX = xClamped;
    this.prevY = yClamped;

    var rotRaw = rotThrottle.getAsDouble();
    var rotScaled = rotRamp.calculate(rotRaw * this.maxRotSpeed);
    return new ChassisSpeeds(xClamped, yClamped, rotScaled);
  }
}
