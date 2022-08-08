package frc.team449.system.encoder

import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * todo have absolute encoders be separate from relative ones
 *   because this velocity calculation is likely not right
 * <p>
 * An absolute encoder masquerading as a relative encoder
 * <p>
 * To calculate velocity, it wraps around if it moved by more than half the
 * total distance
 * @param endPos Where the position wraps around (e.g. 2pi)
 */
class AbsoluteEncoder(
  name: String,
  private val enc: DutyCycleEncoder,
  private val endPos: Double,
  unitPerRotation: Double,
  gearing: Double,
  private val inverted: Boolean
) : Encoder(name, 1, unitPerRotation, gearing) {

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN

  override fun getPositionNative(): Double {
    val pos = enc.absolutePosition // TODO should this be absolutePosition or distance?
    if (!this.inverted) {
      return pos
    } else {
      return -pos
    }
  }

  override fun getVelocityNative(): Double {
    val currPos = this.getPositionNative()
    val currTime = Timer.getFPGATimestamp()

    val vel =
      if (prevPos.isNaN()) {
        0.0
      } else {
        val dt = currTime - prevTime
        var dx = currPos - prevPos
        if (dx < -this.endPos / 2) {
          // If it moved by more than half, assume it actually went the other
          // way and wrap around instead
          dx = currPos - prevPos + this.endPos
        }
        dx / dt
      }
    this.prevPos = currPos
    this.prevTime = currTime

    return vel
  }

  companion object {
    /**
     *
     * @param <T>
     * @param channel
     * @param endPos The total distance that the motor can travel (e.g. 2pi rads)
     * @param offset The position that the absolute encoder is actually at when it reads 0
     */
    fun <T : MotorController> creator(
      channel: Int,
      endPos: Double,
      offset: Double,
      unitPerRotation: Double,
      gearing: Double
    ): EncoderCreator<T> =
      EncoderCreator { name, _, inverted ->
        val enc = AbsoluteEncoder(
          name,
          DutyCycleEncoder(channel),
          endPos,
          unitPerRotation,
          gearing,
          inverted
        )
        enc.resetPosition(offset)
        enc
      }
  }
}
