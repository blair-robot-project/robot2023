package frc.team449.system.encoder;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.jetbrains.annotations.NotNull;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * An encoder plugged into a TalonSRX
 */
public final class TalonEncoder extends Encoder {
  private final TalonSRX talon;

  private TalonEncoder(@NotNull String name, @NotNull TalonSRX talon, int encoderCPR,
      double unitPerRotation, double gearing) {
    // The Talon multiplies its encoder count by 4
    super(name, encoderCPR * 4, unitPerRotation, gearing);
    this.talon = talon;
    this.resetPosition(0);
  }

  public static <T extends MotorController> EncoderCreator<T> creator(
      @NotNull TalonSRX talon,
      int encoderCPR,
      double unitPerRotation,
      double gearing) {
    return (__, config) -> new TalonEncoder(
        config.getEncName(), talon, encoderCPR, unitPerRotation, gearing);
  }

  @Override
  public double getPositionNative() {
    return this.talon.getSelectedSensorPosition(0);
  }

  @Override
  public double getVelocityNative() {
    return this.talon.getSelectedSensorVelocity(0);
  }
}
