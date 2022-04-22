package frc.team449.system.encoder;

import java.util.Objects;

/**
 * Basically a dummy encoder object. It doesn't get position and velocity from anywhere, it relies
 * on a {@link SimEncoderController} setting its position and velocity fields periodically.
 *
 * <p>{@link SimEncoder}s should only have to be created inside {@link
 * frc.team449.system.motor.MotorConfig#build()}
 *
 * <p>Make sure that SimEncoders are created only during simulation and that only SimEncoders are
 * created during simulation. If you create, say, a {@link NEOEncoder}, during simulation and try to
 * use a {@link SimEncoderController} to control it, the {@link SimEncoderController} constructor
 * will throw an error.
 */
public class SimEncoder extends Encoder {
  private double position;
  private double velocity;

  /** Whether a controller has been registered for this simulated encoder */
  private boolean claimed;

  public SimEncoder(String name) {
    super(name, 1, 1, 1);
  }

  @Override
  protected double getPositionNative() {
    return position;
  }

  @Override
  protected double getVelocityNative() {
    return velocity;
  }

  private void setPosition(double pos) {
    this.position = pos;
  }

  private void setVelocity(double vel) {
    this.velocity = vel;
  }

  /** Register the controller that will set the position and velocity for this simulated encoder. */
  private synchronized void registerController(SimEncoderController controller) {
    Objects.requireNonNull(controller);
    if (this.claimed) {
      throw new IllegalStateException(
          "SimEncoder"
              + this.configureLogName()
              + " already has a controller. There can only be one.");
    }
    this.claimed = true;
  }

  /**
   * Used to control {@link SimEncoder}s. Only one {@link SimEncoderController} should be used per
   * {@link SimEncoder}.
   *
   * <p>Make sure to only create SimEncoders during simulation. If you create, say, a {@link
   * NEOEncoder}, during simulation and try to use a {@link SimEncoderController} to control it, the
   * {@link SimEncoderController} constructor will throw an error.
   */
  public static final class SimEncoderController {
    private final SimEncoder encoder;

    public SimEncoderController(SimEncoder encoder) {
      encoder.registerController(this);
      this.encoder = encoder;
    }

    /** Set the position of the {@link SimEncoder} object this is controlling. */
    public void setPosition(double pos) {
      encoder.setPosition(pos);
    }

    /** Set the velocity of the {@link SimEncoder} object this is controlling. */
    public void setVelocity(double vel) {
      encoder.setVelocity(vel);
    }
  }
}
