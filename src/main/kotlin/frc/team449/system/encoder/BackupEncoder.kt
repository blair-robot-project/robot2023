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
class BackupEncoder(
  val primary: Encoder,
  val  fallback: Encoder
  val velThreshold: Double): Encoder(primary.configureLogName(), 1.0, 1.0, 1.0) {

  /** Whether the primary encoder's stopped working */
  @Log private var useFallback = false;

  protected fun getPositionNative(): Double {
    if (useFallback) {
      return fallback.getPosition();
    } else {
      return primary.getPosition();
    }
  }

  protected fun getVelocityNative(): Double {
    val fallbackVel = fallback.getVelocity();
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

  companion object {
    fun <T : MotorController> creator(
       primaryCreator: EncoderCreator<T>,
        fallbackCreator: EncoderCreator<T>,
         velThreshold: Double): EncoderCreator<T>  = {
    (name, motor, inverted) ->
        BackupEncoder(
            primaryCreator.create(name, motor, inverted),
            fallbackCreator.create(name, motor, inverted),
            velThreshold)
    }
  }
}
