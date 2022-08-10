package frc.team449.system.encoder

import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.motorcontrol.MotorController

/**
 * This class uses an absolute encoder, gear ratio and UPR to give the absolute position of the module or rotational velocity of the module.
 *
 * @param gearing This is the gear ratio which is the reciprocal of the # of teeth of driving/follower
 */
class AbsoluteEncoder(
  name: String,
  private val enc: DutyCycleEncoder,
  private val unitPerRotation: Double,
  private val gearing: Double,
  private val inverted: Boolean
) : Encoder(name, 1, unitPerRotation, 1.0) {

  private var prevPos = Double.NaN
  private var prevTime = Double.NaN
  private val initAngle = enc.absolutePosition

  /** This returns the absolute position of the module using gearing and UPR */
  override fun getPositionNative(): Double {
    return if (!this.inverted) {
      ((initAngle + enc.distance) * gearing * unitPerRotation) % unitPerRotation
    } else {
      unitPerRotation - (((1 - initAngle - enc.distance) * gearing * unitPerRotation) % unitPerRotation)
    }
  }

  /** This returns the rotational velocity (on vertical axis) of the module using gearing and UPR */
  override fun getVelocityNative(): Double {
    val currPos =
      if (!this.inverted) {
        (initAngle + enc.distance) * gearing * unitPerRotation
      }
      else {
        (1 - initAngle - enc.distance) * gearing * unitPerRotation
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
     * @param channel
     * @param offset The position that the absolute encoder is actually at when it reads 0
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
          inverted
        )
        enc.resetPosition(offset)
        enc
      }
  }
}
