package frc.team449.control.holonomic;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class OIHolonomic implements Supplier<ChassisSpeeds> {

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;
  private final SlewRateLimiter rotRamp;
  private final double maxLinearSpeed;
  private final double maxRotSpeed;
  private final double maxAccel;

  private double prevX;
  private double prevY;
  private double prevTime;

  public OIHolonomic(
      DoubleSupplier xThrottle,
      DoubleSupplier yThrottle,
      DoubleSupplier rotThrottle,
      SlewRateLimiter rotRamp,
      double maxLinearSpeed,
      double maxRotSpeed,
      double maxAccel) {
    this.rotThrottle = rotThrottle;
    this.yThrottle = yThrottle;
    this.xThrottle = xThrottle;
    this.rotRamp = rotRamp;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxRotSpeed = maxRotSpeed;
    this.maxAccel = maxAccel;
    this.prevTime = Timer.getFPGATimestamp();
  }

  /** @return The {@link ChassisSpeeds} for the given x, y and rotation input from the joystick */
  @Override
  public ChassisSpeeds get() {
    var currTime = Timer.getFPGATimestamp();
    var dt = currTime - prevTime;
    this.prevTime = currTime;
    var xRaw = xThrottle.getAsDouble();
    var yRaw = yThrottle.getAsDouble();

    // Make sure magnitude doesn't go above 1
    // in case both throttles are near full speed
    var magRaw = Math.min(1, Math.hypot(xRaw, yRaw));
    var angle = Math.atan2(xRaw, yRaw);
    var xRawClamped = Math.cos(angle) * magRaw;
    var yRawClamped = Math.sin(angle) * magRaw;

    var xScaled = xRawClamped * this.maxLinearSpeed;
    var yScaled = yRawClamped * this.maxLinearSpeed;

    // Clamp the translation throttles
    var dx = this.prevX - xScaled;
    var dy = this.prevY - yScaled;
    var magAcc = Math.hypot(dx / dt, dy / dt);
    var magAccClamped = Math.min(this.maxAccel, magAcc);
    var dxClamped = dx * magAccClamped / magAcc;
    var dyClamped = dy * magAccClamped / magAcc;

    var xClamped = prevX + dxClamped * dt;
    var yClamped = prevY + dyClamped * dt;

    this.prevX = xClamped;
    this.prevY = yClamped;

    var rotRaw = rotThrottle.getAsDouble();
    var rotScaled = rotRamp.calculate(rotRaw * this.maxRotSpeed);
    return new ChassisSpeeds(xClamped, yClamped, rotScaled);
  }
}
