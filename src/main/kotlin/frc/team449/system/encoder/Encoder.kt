package frc.team449.system.encoder

import io.github.oblarg.oblog.Loggable

/**
 * A wrapper around encoders. Allows resetting encoders to a position.
 *
 * <p>Don't instantiate its subclasses directly. Instead, use their static creator methods
 * @param encoderCPR Counts per rotation of the encoder
 * @param unitPerRotation Meters traveled per rotation of the motor
 * @param gearing The factor the output changes by after being measured by the encoder (should be
 * ```
 *     >= 1, not a reciprocal), e.g. this would be 70 if there were a 70:1 gearing between the
 *     encoder and the final output
 * ```
 */
abstract class Encoder(
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

  /** Whether or not this encoder is being simulated */
  private var simulated = false

  /** Simulated position set by SimEncoderController */
  private var simPos = 0.0

  /** Simulated position set by SimEncoderController */
  private var simVel = 0.0

  /** Current position in encoder's units */
  protected abstract fun getPositionNative(): Double

  /** Current velocity in encoder's units */
  protected abstract fun getVelocityNative(): Double

  /** Position in meters or whatever unit you set */
  var position: Double
    get() {
      val posUnits = if (simulated) simPos else this.getPositionNative() * encoderToUnit
      return positionOffset + posUnits
    }
    set(pos) {
      this.positionOffset = pos - this.position
    }

  /** Velocity in meters per second or whatever unit you set */
  val velocity: Double
    get() {
      return if (simulated) simVel else this.getVelocityNative() * encoderToUnit
    }

  override fun configureLogName() = this.name

  /**
   * Used to control {@link Encoder}s. Only one {@link SimEncoderController} can be used per encoder
   * object.
   *
   * @param enc The encoder to control.
   */
  class SimController(private val enc: Encoder) {
    init {
      if (enc.simulated) {
        throw IllegalStateException("${enc.name} is already being simulated.")
      }
      enc.simulated = true
    }

    /** Set the position of the {@link SimEncoder} object this is controlling. */
    var position: Double
      get() {
        return enc.simPos
      }
      set(pos) {
        enc.simPos = pos
      }

    /** Set the velocity of the {@link SimEncoder} object this is controlling. */
    var velocity: Double
      get() {
        return enc.simVel
      }
      set(vel) {
        enc.simVel = vel
      }
  }
}
