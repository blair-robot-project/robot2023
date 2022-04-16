package frc.team449.system.encoder;

import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.NotNull;

/**
 * A wrapper around encoders. Allows resetting encoders to a position.
 *
 * <p>Don't instantiate its subclasses directly. Instead, use their static creator methods
 */
public abstract class Encoder implements Loggable {
  private final String name;
  /**
   * Factor to multiply by to turn native encoder units into meters or whatever units are actually
   * wanted
   */
  private final double encoderToUnit;
  /** An offset added to the position to allow resetting position. */
  private double positionOffset;

  /**
   * @param encoderCPR Counts per rotation of the encoder
   * @param unitPerRotation Meters traveled per rotation of the motor
   * @param gearing The factor the output changes by after being measured by the encoder (should be
   *     >= 1, not a reciprocal), e.g. this would be 70 if there were a 70:1 gearing between the
   *     encoder and the final output
   */
  public Encoder(@NotNull String name, int encoderCPR, double unitPerRotation, double gearing) {
    this.name = name;
    this.encoderToUnit = unitPerRotation * gearing / encoderCPR;
    this.resetPosition(0);
  }

  /**
   * Current position in encoder's units
   *
   * @see Encoder#getPosition()
   */
  protected abstract double getPositionNative();

  /**
   * Current velocity in encoder's units
   *
   * @see Encoder#getVelocity()
   */
  protected abstract double getVelocityNative();

  /** Current position in meters */
  public final double getPosition() {
    return positionOffset + this.getPositionNative() * encoderToUnit;
  }

  /** Current velocity in meters per second */
  public final double getVelocity() {
    return this.getVelocityNative() * encoderToUnit;
  }

  public void resetPosition(double pos) {
    this.positionOffset = pos - this.getPosition();
  }

  @Override
  public final String configureLogName() {
    return this.name;
  }
}
