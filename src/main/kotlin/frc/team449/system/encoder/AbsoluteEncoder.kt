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
) : Encoder(name, 1, unitPerRotation, 1.0) {
  init {
  }

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN

  /** This returns the absolute position of the module using gearing and UPR and includes offsetting */
  @Log
  override fun getPositionNative(): Double {
    return if (inverted) {
      1 - enc.absolutePosition
    } else {
      enc.absolutePosition
    }
  }

  /** This returns the rotational velocity (on vertical axis) of the module using gearing and UPR */
  @Log
  override fun getVelocityNative(): Double {
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
     */
    fun <T : MotorController> creator(
      channel: Int,
      offset: Double,
      unitPerRotation: Double,
    ): EncoderCreator<T> =
      EncoderCreator { name, _, inverted ->
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
