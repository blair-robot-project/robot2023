package frc.team449.system.encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.jetbrains.annotations.NotNull;

/** A NEO integrated encoder plugged into a Spark */
public final class NEOEncoder extends Encoder {
  public static final int NEO_ENCODER_CPR = 1;

  private final RelativeEncoder enc;

  private NEOEncoder(
      @NotNull String name, @NotNull RelativeEncoder enc, double unitPerRotation, double gearing) {
    super(name, NEO_ENCODER_CPR, 1, 1);
    // Let the underlying encoder do the conversions
    enc.setPositionConversionFactor(unitPerRotation * gearing);
    enc.setVelocityConversionFactor(unitPerRotation * gearing / 60);
    this.enc = enc;
  }

  public static EncoderCreator<CANSparkMax> creator(double unitPerRotation, double gearing) {
    return (name, motor, inverted) ->
        new NEOEncoder(name, motor.getEncoder(), unitPerRotation, gearing);
  }

  @Override
  public double getPositionNative() {
    return enc.getPosition();
  }

  @Override
  public double getVelocityNative() {
    return enc.getVelocity();
  }
}
