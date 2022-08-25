package frc.team449.system.encoder

import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import io.github.oblarg.oblog.annotations.Log

/**
 * This class uses an absolute encoder, gear ratio and UPR to give the absolute position of the module or rotational velocity of the module.
 *
 * @param offset This must be in rotations of how much the offset of the ENCODER should be. TODO: Test what does setPositionOffset actually does, write description here.
 */
class AbsoluteEncoder(
  name: String,
  private val enc: DutyCycleEncoder,
  private val unitPerRotation: Double,
  private val inverted: Boolean,
  private val offset: Double,
  pollTime: Double = .02
) : Encoder(name, 1, unitPerRotation, 1.0, pollTime) {
  private var prevPos = Double.NaN
  private var prevTime = Double.NaN

  /** This returns the absolute position of the module */
  @Log
  override fun getPositionNative(): Double {
    return if (inverted) {
      1 - (enc.absolutePosition - offset) % 1
    } else {
      (enc.absolutePosition - offset) % 1
    }
  }

  /** This returns the rotational velocity (on vertical axis) of the module */
  override fun pollVelocityNative(): Double {
    val currPos =
      if (inverted) {
        -enc.distance
      } else {
        enc.distance
      }

    val currTime = Timer.getFPGATimestamp()

    val vel =
      if (prevPos.isNaN()) {
        0.0
      } else {
        val dt = currTime - prevTime
        val dx = currPos - prevPos
        dx / dt
      }
    this.prevPos = currPos
    this.prevTime = currTime

    return vel
  }

  companion object {
    /**
     * @param <T>
     * @param channel The DutyCycleEncoder port
     * @param offset The position to put into DutyCycleEncoder's setPositionOffset
     * @param unitPerRotation units measured when done one rotation (e.g 360 degrees per rotation)
     * @param inverted If the encoder needs to be inverted or not
     */
    fun <T : MotorController> creator(
      channel: Int,
      offset: Double,
      unitPerRotation: Double,
      inverted: Boolean
    ): EncoderCreator<T> =
      EncoderCreator { name, _, _ ->
        val enc = AbsoluteEncoder(
          name,
          DutyCycleEncoder(channel),
          unitPerRotation,
          inverted,
          offset
        )
        enc
      }
  }
}
