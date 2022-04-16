package frc.team449.system.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.team449.system.encoder.EncoderCreator;
import frc.team449.system.encoder.SimEncoder;
import io.github.oblarg.logexample.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * todo find a better way to do this than F-bounds
 *
 * <p>The constructor for SmartMotors was hell so this will help resolve that.
 *
 * <p>You can set config options in this then pass it to various constructors.
 *
 * @param <Self> The type of the "current" subclass of {@link MotorConfig}
 * @param <R> The type of the created motor
 * @see WrappedMotor
 */
public abstract class MotorConfig<Self extends MotorConfig<Self, R>, R extends MotorController> {
  private String name;
  private boolean enableBrakeMode;
  private int port = -1;
  private boolean inverted;
  private @Nullable Boolean fwdLimitSwitchNormallyOpen;
  private @Nullable Boolean revLimitSwitchNormallyOpen;
  private @Nullable Integer remoteLimitSwitchID;
  private @Nullable Integer currentLimit;
  private boolean enableVoltageComp;
  private EncoderCreator<R> encoderCreator;

  public String getName() {
    return name;
  }

  public Self setName(String name) {
    this.name = name;
    return (Self) this;
  }

  /** The name that the encoder for this motor uses. The motor's name itself should be set first. */
  public String getEncName() {
    return this.getName() + "Enc";
  }

  public int getPort() {
    return port;
  }

  public Self setPort(int port) {
    this.port = port;
    return (Self) this;
  }

  public boolean isEnableBrakeMode() {
    return enableBrakeMode;
  }

  public Self setEnableBrakeMode(boolean enableBrakeMode) {
    this.enableBrakeMode = enableBrakeMode;
    return (Self) this;
  }

  public boolean isInverted() {
    return inverted;
  }

  public Self setInverted(boolean inverted) {
    this.inverted = inverted;
    return (Self) this;
  }

  public @Nullable Boolean getFwdLimitSwitchNormallyOpen() {
    return fwdLimitSwitchNormallyOpen;
  }

  public Self setFwdLimitSwitchNormallyOpen(Boolean fwdLimitSwitchNormallyOpen) {
    this.fwdLimitSwitchNormallyOpen = fwdLimitSwitchNormallyOpen;
    return (Self) this;
  }

  public @Nullable Boolean getRevLimitSwitchNormallyOpen() {
    return revLimitSwitchNormallyOpen;
  }

  public Self setRevLimitSwitchNormallyOpen(Boolean revLimitSwitchNormallyOpen) {
    this.revLimitSwitchNormallyOpen = revLimitSwitchNormallyOpen;
    return (Self) this;
  }

  public @Nullable Integer getRemoteLimitSwitchID() {
    return remoteLimitSwitchID;
  }

  public Self setRemoteLimitSwitchID(Integer remoteLimitSwitchID) {
    this.remoteLimitSwitchID = remoteLimitSwitchID;
    return (Self) this;
  }

  public @Nullable Integer getCurrentLimit() {
    return currentLimit;
  }

  public Self setCurrentLimit(Integer currentLimit) {
    this.currentLimit = currentLimit;
    return (Self) this;
  }

  public boolean isEnableVoltageComp() {
    return enableVoltageComp;
  }

  public Self setEnableVoltageComp(boolean enableVoltageComp) {
    this.enableVoltageComp = enableVoltageComp;
    return (Self) this;
  }

  /** Set the function used to create an encoder once the motor controller is made */
  public Self setEncoderCreator(EncoderCreator<R> encoderCreator) {
    this.encoderCreator = encoderCreator;
    return (Self) this;
  }

  /** Copy properties from this config to another config */
  protected final void copyTo(@NotNull MotorConfig<?, R> other) {
    other
        .setName(name)
        .setEnableBrakeMode(enableBrakeMode)
        .setInverted(inverted)
        .setFwdLimitSwitchNormallyOpen(fwdLimitSwitchNormallyOpen)
        .setRevLimitSwitchNormallyOpen(revLimitSwitchNormallyOpen)
        .setRemoteLimitSwitchID(remoteLimitSwitchID)
        .setCurrentLimit(currentLimit)
        .setEnableVoltageComp(enableVoltageComp)
        .setEncoderCreator(encoderCreator);
  }

  /** Create only the underlying motor controller, not the encoder. */
  @NotNull
  protected abstract R createMotor();

  /**
   * Create a WrappedMotor with all the properties configured previously. Make sure that all
   * required properties have been set before calling this method.
   */
  @NotNull
  public final WrappedMotor build() {
    var motor = this.createMotor();
    var encoder =
        Robot.isReal() ? encoderCreator.create(motor, this) : new SimEncoder(getEncName());
    return new WrappedMotor(name, motor, encoder);
  }
}
