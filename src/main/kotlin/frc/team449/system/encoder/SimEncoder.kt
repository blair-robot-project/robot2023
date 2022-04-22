package frc.team449.system.encoder

import java.util.Objects

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
class SimEncoder(name: String) : Encoder(name, 1, 1.0, 1.0) {
  private var simPosition = 0.0
  private var simVelocity = 0.0

  /** Whether a controller has been registered for this simulated encoder */
  private var claimed = false

  override fun getPositionNative() = simPosition

  override fun getVelocityNative() = simVelocity

  /** Register the controller that will set the position and velocity for this simulated encoder. */
  @Synchronized
  private fun registerController(controller: SimEncoderController) {
    Objects.requireNonNull(controller)
    if (this.claimed) {
      throw IllegalStateException(
        "SimEncoder" +
          this.configureLogName() +
          " already has a controller. There can only be one."
      )
    }
    this.claimed = true
  }

  companion object {
/**
     * Used to control {@link SimEncoder}s. Only one {@link SimEncoderController} should be used per
     * {@link SimEncoder}.
     *
     * <p>Make sure to only create SimEncoders during simulation. If you create, say, a {@link
     * NEOEncoder}, during simulation and try to use a {@link SimEncoderController} to control it, the
     * {@link SimEncoderController} constructor will throw an error.
     */
    class SimEncoderController(private val encoder: SimEncoder) {
      init {
        encoder.registerController(this)
      }

      /** Set the position of the {@link SimEncoder} object this is controlling. */
      var position: Double
        get() {
          return encoder.simPosition
        }

        set(pos) {
          encoder.simPosition = pos
        }

      /** Set the velocity of the {@link SimEncoder} object this is controlling. */
      var velocity: Double
        get() {
          return encoder.simVelocity
        }

        set(vel) {
          encoder.simVelocity = vel
        }
    }
  }
}
