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
 * <p> The magnitude of the acceleration is clamped </p>
 */
public class OIHolonomic implements Supplier<ChassisSpeeds> {

  private final HolonomicDrive drive;
  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;
  private final SlewRateLimiter rotRamp;
  private final double maxAccel;
  private final boolean fieldOriented;

  private double prevX;
  private double prevY;
  private double prevTime = Double.NaN;

  /**
   * Create an OI for a holonomic drivetrain. Note that the joystick's X
   * axis corresponds to the robot's/field's Y and vice versa
   *
   * @param drive The drivetrain this OI is controlling
   * @param xThrottle The Y axis of the strafing joystick
   * @param yThrottle The X axis of the strafing joystick
   * @param rotThrottle
   * @param rotRamp Used to ramp angular velocity
   * @param maxAccel Max accel, used for ramping
   * @param fieldOriented Whether the OI x and y translation should
   * be relative to the field rather than relative to the robot. This better be true.
   */
  public OIHolonomic(
    HolonomicDrive drive,
    DoubleSupplier xThrottle,
    DoubleSupplier yThrottle,
    DoubleSupplier rotThrottle,
    SlewRateLimiter rotRamp,
    double maxAccel,
    boolean fieldOriented
  ) {
    this.drive = drive;
    this.rotThrottle = rotThrottle;
    this.yThrottle = yThrottle;
    this.xThrottle = xThrottle;
    this.rotRamp = rotRamp;
    this.maxAccel = maxAccel;
    this.fieldOriented = fieldOriented;
  }

  /**
   * @return The {@link ChassisSpeeds} for the given x, y and
   * rotation input from the joystick */
  @Override
  public ChassisSpeeds get() {
    var currTime = Timer.getFPGATimestamp();
    if (Double.isNaN(this.prevTime)) {
      this.prevTime = currTime - 0.02;
    }
    var dt = currTime - prevTime;
    this.prevTime = currTime;

    var xScaled = xThrottle.getAsDouble() * drive.getMaxLinearSpeed();
    var yScaled = yThrottle.getAsDouble() * drive.getMaxLinearSpeed();

    // Ramp the translation throttles
    var dx = this.prevX - xScaled;
    var dy = this.prevY - yScaled;
    var maxAcc = Math.hypot(dx / dt, dy / dt);
    var maxAccClamped = MathUtil.clamp(maxAcc, -this.maxAccel, this.maxAccel);
    var dxClamped = dx * maxAccClamped / maxAcc;
    var dyClamped = dy * maxAccClamped / maxAcc;

    var xClamped = prevX + dxClamped * dt;
    var yClamped = prevY + dyClamped * dt;

    this.prevX = xClamped;
    this.prevY = yClamped;

    var rotRaw = rotThrottle.getAsDouble();
    var rotScaled = rotRamp.calculate(rotRaw * drive.getMaxRotSpeed());

    if (this.fieldOriented) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
        xClamped,
        yClamped,
        rotScaled,
        drive.getHeading()
      );
    } else {
      return new ChassisSpeeds(xClamped, yClamped, rotScaled);
    }
  }
}
