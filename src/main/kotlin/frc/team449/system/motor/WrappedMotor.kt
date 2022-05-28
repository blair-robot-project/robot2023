package frc.team449.system.motor

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import frc.team449.system.encoder.Encoder
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

/** Our own wrapper grouping the motor controller and encoder for a motor
 *
 * @param name Name for logging
 */
class WrappedMotor(
  private val name: String,
  private val motor: MotorController,
  val encoder: Encoder
) : MotorController by motor, Loggable {
  @Log
  var voltage = 0.0
    private set

  /** Position in meters or whatever unit you set */
  var position: Double
    get() = encoder.position
    set(pos) {
      encoder.position = pos
    }

  /** Velocity in meters per second or whatever unit you set */
  val velocity: Double
    get() = encoder.velocity

  override fun setVoltage(volts: Double) {
    motor.setVoltage(volts)
    this.voltage = volts
  }

  override fun set(output: Double) {
    motor.set(output)
    this.voltage = output * RobotController.getBatteryVoltage()
  }

  override fun configureLogName() = this.name
}
