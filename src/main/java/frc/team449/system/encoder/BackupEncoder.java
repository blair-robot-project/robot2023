package frc.team449.system.encoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

/**
 * A wrapper to use when you have one external encoder that's more accurate but may be unplugged and
 * an integrated encoder that's less accurate but is less likely to be unplugged.
 *
 * <p>If the primary encoder's velocity is 0 but the integrated encoder's is above a given
 * threshold, it concludes that the primary encoder is broken and switches to using the
 * fallback/integrated encoder.
 */
public class BackupEncoder extends Encoder {

  @NotNull private final Encoder primary;
  @NotNull private final Encoder fallback;
  private final double velThreshold;

  /** Whether the primary encoder's stopped working */
  @Log private boolean useFallback = false;

  private BackupEncoder(@NotNull Encoder primary, @NotNull Encoder fallback, double velThreshold) {
    super(primary.configureLogName(), 1, 1, 1);

    this.primary = primary;
    this.fallback = fallback;
    this.velThreshold = velThreshold;
  }

  public static <T extends MotorController> EncoderCreator<T> creator(
      EncoderCreator<T> primaryCreator, EncoderCreator<T> fallbackCreator, double velThreshold) {
    return (name, motor, inverted) ->
        new BackupEncoder(
            primaryCreator.create(name, motor, inverted),
            fallbackCreator.create(name, motor, inverted),
            velThreshold);
  }

  @Override
  protected double getPositionNative() {
    if (useFallback) {
      return fallback.getPosition();
    } else {
      return primary.getPosition();
    }
  }

  @Override
  protected double getVelocityNative() {
    var fallbackVel = fallback.getVelocity();
    if (useFallback) {
      return fallbackVel;
    } else {
      var primaryVel = primary.getVelocity();
      if (primaryVel == 0 && Math.abs(fallbackVel) > velThreshold) {
        this.useFallback = true;
        return fallbackVel;
      } else {
        return primaryVel;
      }
    }
  }
}
