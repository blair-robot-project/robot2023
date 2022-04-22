package frc.team449.system.encoder

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import org.jetbrains.annotations.NotNull

/** An external quadrature encoder */
class QuadEncoder(
  name: String,
  private val encoder: edu.wpi.first.wpilibj.Encoder,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double
  ) QuadEncoder : Encoder(name, 1.0, 1.0, 1.0) {
  init {
    // Let the WPI encoder handle the distance scaling
    encoder.setDistancePerPulse(unitPerRotation * gearing / encoderCPR)
    encoder.setSamplesToAverage(5)
    this.encoder = encoder
  }

  fun getPositionNative() =  encoder.getDistance()

  fun getVelocityNative() = encoder.getRate()

  companion object {
    fun <T : MotorController>  creator(
       encoder: edu.wpi.first.wpilibj.Encoder,
       encoderCPR: Int,
      unitPerRotation: Double,
      gearing: Double): EncoderCreator<T> = {
    (name, motor, inverted) -> {
      encoder.setReverseDirection(inverted)
      return new QuadEncoder(name, encoder, encoderCPR, unitPerRotation, gearing)
    }
    }
  }
}
