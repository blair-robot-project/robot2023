package frc.team449.system.motor

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.team449.system.encoder.Encoder
import io.github.oblarg.oblog.Loggable

/** Our own wrapper grouping the motor controller and encoder for a motor
 * @param name Name for logging
 */
class WrappedMotor(
  private val name: String,
  private val motor: MotorController,
  private val encoder: Encoder
) : MotorController by motor, Loggable {
  /** Position in meters or whatever unit you set */
  var position: Double
    get() {
      return encoder.position
    }
    set(pos) {
      encoder.position = pos
    }

  /** Velocity in meters per second or whatever unit you set */
  val velocity: Double
    get() {
      return encoder.velocity
    }

  override fun configureLogName() = this.name
}
