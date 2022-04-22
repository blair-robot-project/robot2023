package frc.team449.system.encoder

import io.github.oblarg.oblog.Loggable

/**
 * A wrapper around encoders. Allows resetting encoders to a position.
 *
 * <p>Don't instantiate its subclasses directly. Instead, use their static creator methods
 * @param encoderCPR Counts per rotation of the encoder
 * @param unitPerRotation Meters traveled per rotation of the motor
 * @param gearing The factor the output changes by after being measured by the encoder (should be
 *     >= 1, not a reciprocal), e.g. this would be 70 if there were a 70:1 gearing between the
 *     encoder and the final output
 */
open class Encoder(
  val name: String,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double
) : Loggable {
  /**
   * Factor to multiply by to turn native encoder units into meters or whatever units are actually
   * wanted
   */
  val encoderToUnit = unitPerRotation * gearing / encoderCPR
  /** An offset added to the position to allow resetting position. */
  var positionOffset = 0.0

  /**
   * Current position in encoder's units
   *
   * @see Encoder#getPosition()
   */
  protected abstract fun getPositionNative(): Double

  /**
   * Current velocity in encoder's units
   *
   * @see Encoder#getVelocity()
   */
  protected abstract fun getVelocityNative(): Double

  /** Current position in meters */
  val position: Double
    get() {
      return positionOffset + this.getPositionNative() * encoderToUnit
    }
    set(pos) {
      this.positionOffset = pos - this.getPosition()
    }

  /** Current velocity in meters per second */
  val velocity: Double
    get() {
      return this.getVelocityNative() * encoderToUnit
    }

  override fun configureLogName() = this.name
}
