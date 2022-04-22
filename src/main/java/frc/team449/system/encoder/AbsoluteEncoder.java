package frc.team449.system.encoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.jetbrains.annotations.NotNull;

/**
 * todo have absolute encoders be separate from relative ones
 *   because this velocity calculation is likely not right
 * <p>
 * An absolute encoder masquerading as a relative encoder
 * <p>
 * To calculate velocity, it wraps around if it moved by more than half the
 * total distance
 */
public class AbsoluteEncoder extends Encoder {

  private final AnalogEncoder enc;
  private final double endPos;
  private final boolean inverted;

  private double prevPos = Double.NaN;
  private double prevTime = Double.NaN;

  /**
   * @param enc Actual analog encoder object to wrap
   * @param endPos Where the position wraps around (e.g. 2pi)
   */
  private AbsoluteEncoder(
    @NotNull String name,
    AnalogEncoder enc,
    double endPos,
    double unitPerRotation,
    double gearing,
    boolean inverted
  ) {
    super(name, 1, 1, 1);
    enc.setDistancePerRotation(unitPerRotation * gearing);
    this.enc = enc;
    this.endPos = endPos;
    this.inverted = inverted;
  }

  /**
   *
   * @param <T>
   * @param channel
   * @param endPos The total distance that the motor can travel (e.g. 2pi rads)
   * @param offset The position that the absolute encoder is actually at when it reads 0
   */
  public static <T extends MotorController> EncoderCreator<T> creator(
    int channel,
    double endPos,
    double offset,
    double unitPerRotation,
    double gearing
  ) {
    return (name, motor, inverted) -> {
      var enc = new AbsoluteEncoder(
        name,
        new AnalogEncoder(channel),
        endPos,
        unitPerRotation,
        gearing,
        inverted
      );
      enc.resetPosition(offset);
      return enc;
    };
  }

  @Override
  protected double getPositionNative() {
    var pos = enc.getAbsolutePosition();
    if (!this.inverted) {
      return pos;
    } else {
      return this.endPos - pos;
    }
  }

  @Override
  protected double getVelocityNative() {
    var currPos = this.getPositionNative();
    var currTime = Timer.getFPGATimestamp();

    double vel;
    if (Double.isNaN(prevPos)) {
      vel = 0;
    } else {
      var dt = currTime - prevTime;
      var dx = currPos - prevPos;
      if (dx < -this.endPos / 2) {
        // If it moved by more than half, assume it actually went the other
        // way and wrap around instead
        dx = currPos - prevPos + this.endPos;
      }
      vel = dx / dt;
    }

    this.prevPos = currPos;
    this.prevTime = currTime;

    return vel;
  }
}
