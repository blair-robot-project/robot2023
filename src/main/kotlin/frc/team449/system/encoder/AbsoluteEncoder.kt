package frc.team449.system.encoder

import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * This class uses an absolute encoder, gear ratio and UPR to give the absolute position of the module or rotational velocity of the module.
 *
 * @param gearing This is the # of teeth of driving/follower
 * @param offset This must be in rotations of how much the offset of the ENCODER should be. TODO: Test what does setPositionOffset actually does, write description here.
 */
class AbsoluteEncoder(
  name: String,
  private val enc: DutyCycleEncoder,
  unitPerRotation: Double,
  private val gearing: Double,
  private val inverted: Boolean,
  private val offset: Double,
) : Encoder(name, 1, unitPerRotation, 1.0) {

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN
  private val initAngle = enc.absolutePosition
  private val invertedInitAngle = 1 - enc.absolutePosition

  /** This returns the absolute position of the module using gearing and UPR and includes offsetting */
  override fun getPositionNative(): Double {
    enc.positionOffset = offset
    return if (!this.inverted) {
      ((initAngle + enc.distance) * gearing) % 1
    } else {
      ((invertedInitAngle - enc.distance) * gearing) % 1
    }
  }

  /** This returns the rotational velocity (on vertical axis) of the module using gearing and UPR */
  override fun getVelocityNative(): Double {
    val currPos =
      if (!this.inverted) {
        (enc.distance) * gearing
      } else {
        (-enc.distance) * gearing
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
     *
     * @param <T>
     * @param channel The DutyCycleEncoder port
     * @param offset The position to put into DutyCycleEncoder's setPositionOffset
     */
    fun <T : MotorController> creator(
      channel: Int,
      offset: Double,
      unitPerRotation: Double,
      gearing: Double
    ): EncoderCreator<T> =
      EncoderCreator { name, _, inverted ->
        val enc = AbsoluteEncoder(
          name,
          DutyCycleEncoder(channel),
          unitPerRotation,
          gearing,
          inverted,
          offset
        )
        enc
      }
  }
}
