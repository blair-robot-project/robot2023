package frc.team449.system.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxLimitSwitch;
import java.util.HashMap;
import java.util.Map;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** Motor controller configuration, along with some Spark-specific stuff */
public final class SparkMaxConfig extends MotorConfig<SparkMaxConfig, CANSparkMax> {
  private final Map<CANSparkMax.PeriodicFrame, Integer> statusFrameRatesMillis = new HashMap<>();
  private final @NotNull Map<CANSparkMax, Boolean> slaveSparks = new HashMap<>();
  private @Nullable Integer controlFrameRateMillis;

  @Nullable
  public Integer getControlFrameRateMillis() {
    return this.controlFrameRateMillis;
  }

  public SparkMaxConfig setControlFrameRateMillis(int controlFrameRateMillis) {
    this.controlFrameRateMillis = controlFrameRateMillis;
    return this;
  }

  public Map<CANSparkMax.PeriodicFrame, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public SparkMaxConfig addStatusFrameRateMillis(
      CANSparkMaxLowLevel.PeriodicFrame frame, int rate) {
    this.statusFrameRatesMillis.put(frame, rate);
    return this;
  }

  public @NotNull Map<CANSparkMax, Boolean> getSlaveSparks() {
    return slaveSparks;
  }

  /**
   * Add a slave spark
   *
   * @param inverted Whether or not it's inverted
   */
  public SparkMaxConfig addSlaveSpark(@NotNull CANSparkMax slaveSpark, boolean inverted) {
    this.slaveSparks.put(slaveSpark, inverted);
    return this;
  }

  public SparkMaxConfig copy() {
    var copy = new SparkMaxConfig();
    this.copyTo(copy);

    if (this.controlFrameRateMillis != null) {
      copy.setControlFrameRateMillis(this.controlFrameRateMillis);
    }

    copy.statusFrameRatesMillis.putAll(this.getStatusFrameRatesMillis());

    return copy;
  }

  @NotNull
  @Override
  public CANSparkMax createMotor() {
    var motor = new CANSparkMax(this.getPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
    if (motor.getLastError() != REVLibError.kOk) {
      System.out.println(
          "Motor could not be constructed on port "
              + this.getPort()
              + " due to error "
              + motor.getLastError());
    }

    motor.restoreFactoryDefaults();

    var brakeMode =
        this.isEnableBrakeMode() ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;

    motor.setInverted(this.isInverted());
    // Set brake mode
    motor.setIdleMode(brakeMode);

    // Set frame rates
    if (this.getControlFrameRateMillis() != null) {
      // Must be between 1 and 100 ms.
      motor.setControlFramePeriodMs(this.getControlFrameRateMillis());
    }

    this.getStatusFrameRatesMillis().forEach(motor::setPeriodicFramePeriod);

    // todo handle limit switches
    if (this.getFwdLimitSwitchNormallyOpen() == null) {
      motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    }
    if (this.getRevLimitSwitchNormallyOpen() == null) {
      motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    }

    // Set the current limit if it was given
    if (this.getCurrentLimit() != null) {
      motor.setSmartCurrentLimit(this.getCurrentLimit());
    }

    if (this.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(12);
    } else {
      motor.disableVoltageCompensation();
    }

    this.slaveSparks.forEach(
        (slave, inverted) -> {
          slave.restoreFactoryDefaults();
          slave.follow(motor, inverted);
          slave.setIdleMode(brakeMode);
          if (this.getCurrentLimit() != null) {
            slave.setSmartCurrentLimit(this.getCurrentLimit());
          }
          slave.burnFlash();
        });

    motor.burnFlash();

    return motor;
  }
}
