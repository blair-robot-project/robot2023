package frc.team449.system.encoder

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.motorcontrol.MotorController

/** An encoder plugged into a TalonSRX */
class TalonEncoder(
  name: String,
  private val talon: TalonSRX,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double
) : Encoder(name, encoderCPR * 4, unitPerRotation, gearing) {

  override fun getPositionNative() = talon.getSelectedSensorPosition(0)

  override fun getVelocityNative() = talon.getSelectedSensorVelocity(0)

  companion object {
    fun <T : MotorController> creator(
      talon: TalonSRX,
      encoderCPR: Int,
      unitPerRotation: Double,
      gearing: Double
    ): EncoderCreator<T> = EncoderCreator { name, motor, inverted ->
      TalonEncoder(name, talon, encoderCPR, unitPerRotation, gearing)
    }
  }
}
