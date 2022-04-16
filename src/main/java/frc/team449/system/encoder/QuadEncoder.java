package frc.team449.system.encoder;

import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * An external quadrature encoder
 */
public class QuadEncoder extends Encoder {
  private final edu.wpi.first.wpilibj.Encoder encoder;

  private QuadEncoder(
      @NotNull String name,
      @NotNull edu.wpi.first.wpilibj.Encoder encoder,
      int encoderCPR,
      double unitPerRotation,
      double gearing) {
    super(name, 1, 1, 1);
    // Let the WPI encoder handle the distance scaling
    encoder.setDistancePerPulse(unitPerRotation * gearing / encoderCPR);
    encoder.setSamplesToAverage(5);
    this.encoder = encoder;
  }

  public static <T extends MotorController> EncoderCreator<T> creator(
      @NotNull edu.wpi.first.wpilibj.Encoder encoder,
      int encoderCPR,
      double unitPerRotation,
      double gearing) {
    return (__, config) -> {
      encoder.setReverseDirection(config.isInverted());
      return new QuadEncoder(
          config.getEncName(), encoder, encoderCPR, unitPerRotation, gearing);
    };
  }

  @Override
  public double getPositionNative() {
    return encoder.getDistance();
  }

  @Override
  public double getVelocityNative() {
    return encoder.getRate();
  }
}
