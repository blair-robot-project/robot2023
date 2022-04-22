package frc.team449.system.encoder

import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import org.jetbrains.annotations.NotNull

/** A NEO integrated encoder plugged into a Spark */
class NEOEncoder(
  name: String,
private val enc:  RelativeEncoder,
unitPerRotation: Double,
gearing: Double
) : Encoder(name, NEO_ENCODER_CPR, 1.0, 1.0) {

  init {
    // Let the underlying encoder do the conversions
    enc.setPositionConversionFactor(unitPerRotation * gearing)
    enc.setVelocityConversionFactor(unitPerRotation * gearing / 60)
    this.enc = enc
  }

  fun getPositionNative() = enc.getPosition()

  fun getVelocityNative(): Double = enc.getVelocity()

  companion object {
    const val NEO_ENCODER_CPR = 1

    fun  creator(unitPerRotation: Double, gearing: Double): EncoderCreator<CANSparkMax> = {
     (name, motor, inverted) ->
        new NEOEncoder(name, motor.getEncoder(), unitPerRotation, gearing)
    }
  }
}
