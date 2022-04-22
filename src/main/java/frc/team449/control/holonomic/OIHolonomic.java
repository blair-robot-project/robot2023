package frc.team449.control.holonomic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Create an OI for controlling a holonomic drivetrain (probably swerve).
 * The x and y axes on one joystick are used to control x and y velocity (m/s),
 * while the x axis on another joystick is used to control rotational velocity (m/s).
 * <p> The magnitude of the acceleration is clamped </p>
 */
public final class OIHolonomic
    implements Supplier<ChassisSpeeds>, Loggable, Sendable {

  @Log.Exclude
  private final HolonomicDrive drive;

  private final DoubleSupplier xThrottle;
  private final DoubleSupplier yThrottle;
  private final DoubleSupplier rotThrottle;
  private final SlewRateLimiter rotRamp;
  private final double maxAccel;
  private final boolean fieldOriented;

  /** Previous x velocity (scaled and clamped) */
  @Log
  private double prevX;

  /** Previous y velocity (scaled and clamped) */
  @Log
  private double prevY;

  private double prevTime = Double.NaN;

  // todo Remove these soon, only for logging
  private double dx, dy, magAcc, dt;
  private double magAccClamped;

  /**
   * Create an OI for a holonomic drivetrain. Note that the joystick's X
   * axis corresponds to the robot's/field's Y and vice versa
   *
   * @param drive The drivetrain this OI is controlling
   * @param xThrottle The Y axis of the strafing joystick
   * @param yThrottle The X axis of the strafing joystick
   * @param rotThrottle The X axis of the rotating joystick
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
    // SmartDashboard.putData(this.getClass().getSimpleName(), this);
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
    this.dt = currTime - prevTime;
    this.prevTime = currTime;

    var xScaled = xThrottle.getAsDouble() * drive.getMaxLinearSpeed();
    var yScaled = yThrottle.getAsDouble() * drive.getMaxLinearSpeed();

    // Clamp the acceleration
    this.dx = xScaled - this.prevX;
    this.dy = yScaled - this.prevY;
    this.magAcc = Math.hypot(dx / dt, dy / dt);
    this.magAccClamped = MathUtil.clamp(magAcc, -this.maxAccel, this.maxAccel);

    // Scale the change in x and y the same as the acceleration
    var factor = magAcc == 0 ? 0 : magAccClamped / magAcc;
    var dxClamped = dx * factor;
    var dyClamped = dy * factor;
    var xClamped = prevX + dxClamped;
    var yClamped = prevY + dyClamped;

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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("currX", this.xThrottle::getAsDouble, null);
    builder.addDoubleProperty("currY", this.yThrottle::getAsDouble, null);
    builder.addDoubleProperty("prevX", () -> this.prevX, null);
    builder.addDoubleProperty("prevY", () -> this.prevY, null);
    builder.addDoubleProperty("dx", () -> this.dx, null);
    builder.addDoubleProperty("dy", () -> this.dy, null);
    builder.addDoubleProperty("dt", () -> this.dt, null);
    builder.addDoubleProperty("magAcc", () -> this.magAcc, null);
    builder.addDoubleProperty("magAccClamped", () -> this.magAccClamped, null);
    builder.addStringProperty("speeds", () -> this.get().toString(), null);
  }
}
