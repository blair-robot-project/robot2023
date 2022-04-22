package frc.team449.system.encoder

import edu.wpi.first.wpilibj.motorcontrol.MotorController

/** An external quadrature encoder */
class QuadEncoder(
  name: String,
  private val encoder: edu.wpi.first.wpilibj.Encoder,
  encoderCPR: Int,
  unitPerRotation: Double,
  gearing: Double
) : Encoder(name, 1, 1.0, 1.0) {
  init {
    // Let the WPI encoder handle the distance scaling
    encoder.setDistancePerPulse(unitPerRotation * gearing / encoderCPR)
    encoder.setSamplesToAverage(5)
  }

  override fun getPositionNative() = encoder.getDistance()

  override fun getVelocityNative() = encoder.getRate()

  companion object {
    fun <T : MotorController> creator(
      encoder: edu.wpi.first.wpilibj.Encoder,
      encoderCPR: Int,
      unitPerRotation: Double,
      gearing: Double
    ): EncoderCreator<T> =
      EncoderCreator { name, motor, inverted ->
        encoder.setReverseDirection(inverted)
        QuadEncoder(name, encoder, encoderCPR, unitPerRotation, gearing)
      }
  }
}
