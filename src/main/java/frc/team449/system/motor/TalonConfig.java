package frc.team449.system.motor;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/** Motor controller configuration, along with Talon-specific stuff */
public class TalonConfig extends MotorConfig<TalonConfig, WPI_TalonSRX> {

  private final Map<ControlFrame, Integer> controlFrameRatesMillis = new HashMap<>();
  private final Map<StatusFrameEnhanced, Integer> statusFrameRatesMillis = new HashMap<>();
  private final List<TalonSRX> slaveTalons = new ArrayList<>();
  private final List<VictorSPX> slaveVictors = new ArrayList<>();
  private int voltageCompSamples = 32;
  private @Nullable FeedbackDevice feedbackDevice;
  private boolean reverseSensor = false;

  @Override
  protected TalonConfig self() {
    return this;
  }

  @NotNull
  public Map<ControlFrame, Integer> getControlFrameRatesMillis() {
    return new HashMap<>(this.controlFrameRatesMillis);
  }

  public TalonConfig addControlFrameRateMillis(@NotNull ControlFrame controlFrame, int updateRate) {
    this.controlFrameRatesMillis.put(controlFrame, updateRate);
    return this;
  }

  @NotNull
  public Map<StatusFrameEnhanced, Integer> getStatusFrameRatesMillis() {
    return new HashMap<>(this.statusFrameRatesMillis);
  }

  public TalonConfig addStatusFrameRateMillis(
      @NotNull StatusFrameEnhanced statusFrame, int updateRate) {
    this.statusFrameRatesMillis.put(statusFrame, updateRate);
    return this;
  }

  public int getVoltageCompSamples() {
    return this.voltageCompSamples;
  }

  public TalonConfig setVoltageCompSamples(int voltageCompSamples) {
    this.voltageCompSamples = voltageCompSamples;
    return this;
  }

  @Nullable
  public FeedbackDevice getFeedbackDevice() {
    return feedbackDevice;
  }

  public TalonConfig setFeedbackDevice(@NotNull FeedbackDevice feedbackDevice) {
    this.feedbackDevice = feedbackDevice;
    return this;
  }

  public boolean getReverseSensor() {
    return reverseSensor;
  }

  public TalonConfig setReverseSensor(boolean reverseSensor) {
    this.reverseSensor = reverseSensor;
    return this;
  }

  @NotNull
  public List<TalonSRX> getSlaveTalons() {
    return new ArrayList<>(slaveTalons);
  }

  /** @param port The follower's CAN ID */
  public TalonConfig addSlaveTalon(int port, @NotNull InvertType invertType) {
    var talonSRX = new TalonSRX(port);

    // Turn off features we don't want a slave to have
    talonSRX.setInverted(invertType);
    talonSRX.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    talonSRX.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    talonSRX.configForwardSoftLimitEnable(false, 0);
    talonSRX.configReverseSoftLimitEnable(false, 0);
    talonSRX.configPeakOutputForward(1, 0);
    talonSRX.enableVoltageCompensation(true);
    talonSRX.configVoltageCompSaturation(12, 0);
    talonSRX.configVoltageMeasurementFilter(32, 0);

    // Slow down frames so we don't overload the CAN bus
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, 0);
    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 100, 0);

    this.slaveTalons.add(talonSRX);
    return this;
  }

  @NotNull
  public List<VictorSPX> getSlaveVictors() {
    return new ArrayList<>(slaveVictors);
  }

  /**
   * Add a Victor to follow this Talon.
   *
   * @param port The CAN ID of this Victor SPX.
   * @param invertType Whether to invert this relative to the master. Defaults to not inverting
   *     relative to master.
   */
  public TalonConfig addSlaveVictor(int port, @NotNull InvertType invertType) {
    var victorSPX = new VictorSPX(port);
    victorSPX.setInverted(invertType == null ? InvertType.FollowMaster : invertType);
    victorSPX.configPeakOutputForward(1, 0);
    victorSPX.configPeakOutputReverse(-1, 0);
    victorSPX.enableVoltageCompensation(true);
    victorSPX.configVoltageCompSaturation(12, 0);
    victorSPX.configVoltageMeasurementFilter(32, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_1_General, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_6_Misc, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100, 0);
    victorSPX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 100, 0);

    this.slaveVictors.add(victorSPX);
    return this;
  }

  public TalonConfig copy() {
    var copy =
        new TalonConfig()
            .setReverseSensor(this.getReverseSensor())
            .setVoltageCompSamples(this.getVoltageCompSamples());
    if (this.getFeedbackDevice() != null) {
      copy.setFeedbackDevice(this.getFeedbackDevice());
    }
    this.copyTo(copy);

    copy.controlFrameRatesMillis.putAll(this.controlFrameRatesMillis);
    copy.statusFrameRatesMillis.putAll(this.statusFrameRatesMillis);
    copy.slaveTalons.addAll(this.getSlaveTalons());
    copy.slaveVictors.addAll(this.getSlaveVictors());

    return copy;
  }

  @Contract("-> new")
  @NotNull
  @Override
  public WPI_TalonSRX createMotor() {
    var motor = new WPI_TalonSRX(this.getPort());

    motor.setInverted(this.isInverted());
    // Set brake mode
    motor.setNeutralMode(this.isEnableBrakeMode() ? NeutralMode.Brake : NeutralMode.Coast);

    this.getControlFrameRatesMillis().forEach(motor::setControlFramePeriod);
    this.getStatusFrameRatesMillis().forEach(motor::setStatusFramePeriod);

    // Only enable the limit switches if it was specified if they're normally open
    // or closed.
    if (this.getFwdLimitSwitchNormallyOpen() != null) {
      if (this.getRemoteLimitSwitchID() != null) {
        motor.configForwardLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            this.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            this.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            this.getFwdLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configForwardLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }
    if (this.getRevLimitSwitchNormallyOpen() != null) {
      if (this.getRemoteLimitSwitchID() != null) {
        motor.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteTalonSRX,
            this.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            this.getRemoteLimitSwitchID(),
            0);
      } else {
        motor.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            this.getRevLimitSwitchNormallyOpen()
                ? LimitSwitchNormal.NormallyOpen
                : LimitSwitchNormal.NormallyClosed,
            0);
      }
    } else {
      motor.configReverseLimitSwitchSource(
          LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    }

    // Setup feedback device if it exists
    var feedbackDevice = this.getFeedbackDevice();
    if (feedbackDevice != null) {
      // CTRE encoder use RPM instead of native units, and can be used as
      // QuadEncoders, so we switch
      // them to avoid having to support RPM.
      if (feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Absolute
          || feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Relative) {
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
      } else {
        motor.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
      }
      motor.setSensorPhase(this.getReverseSensor());
    } else {
      motor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
    }

    // Set the current limit if it was given
    if (this.getCurrentLimit() != null) {
      motor.configContinuousCurrentLimit(this.getCurrentLimit(), 0);
      motor.configPeakCurrentDuration(0, 0);
      motor.configPeakCurrentLimit(0, 0); // No duration
      motor.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      motor.enableCurrentLimit(false);
    }

    // Enable or disable voltage comp
    if (this.isEnableVoltageComp()) {
      motor.enableVoltageCompensation(true);
      motor.configVoltageCompSaturation(12, 0);
    }
    motor.configVoltageMeasurementFilter(this.getVoltageCompSamples(), 0);

    // Use slot 0
    motor.selectProfileSlot(0, 0);

    // Set up slaves.
    for (var slave : this.getSlaveTalons()) {
      setMasterForTalon(
          slave,
          this.getPort(),
          this.isEnableBrakeMode(),
          this.getCurrentLimit(),
          this.isEnableVoltageComp() ? this.getVoltageCompSamples() : null);
    }

    for (var slave : this.getSlaveVictors()) {
      setMasterForVictor(
          slave,
          motor,
          this.isEnableBrakeMode(),
          this.isEnableVoltageComp() ? this.getVoltageCompSamples() : null);
    }

    motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    motor.configVelocityMeasurementWindow(10);

    // Set max voltage
    motor.configPeakOutputForward(1.0, 0);
    motor.configPeakOutputReverse(1.0, 0);

    // Set min voltage
    // motor.configNominalOutputForward(this.currentGearSettings.fwdNominalOutputVoltage
    // / 12., 0);
    // motor.configNominalOutputReverse(this.currentGearSettings.revNominalOutputVoltage
    // / 12., 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);

    return motor;
  }

  /**
   * Set this Talon to follow another CAN device.
   *
   * @param port The CAN ID of the device to follow.
   * @param brakeMode Whether this Talon should be in brake mode or coast mode.
   * @param currentLimit The current limit for this Talon. Can be null for no current limit.
   * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
   *     compensate voltage.
   */
  private static void setMasterForTalon(
      @NotNull TalonSRX talonSRX,
      final int port,
      final boolean brakeMode,
      @Nullable final Integer currentLimit,
      @Nullable final Integer voltageCompSamples) {
    // Brake mode doesn't automatically follow master
    talonSRX.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    // Current limiting might not automatically follow master, set it just to be
    // safe
    if (currentLimit != null) {
      talonSRX.configContinuousCurrentLimit(currentLimit, 0);
      talonSRX.configPeakCurrentDuration(0, 0);
      talonSRX.configPeakCurrentLimit(0, 0); // No duration
      talonSRX.enableCurrentLimit(true);
    } else {
      // If we don't have a current limit, disable current limiting.
      talonSRX.enableCurrentLimit(false);
    }

    // Voltage comp might not follow master either
    if (voltageCompSamples != null) {
      talonSRX.enableVoltageCompensation(true);
      talonSRX.configVoltageCompSaturation(12, 0);
      talonSRX.configVoltageMeasurementFilter(voltageCompSamples, 0);
    } else {
      talonSRX.enableVoltageCompensation(false);
    }

    // Follow the leader
    talonSRX.set(ControlMode.Follower, port);
  }

  /**
   * Set this Victor to follow another CAN device.
   *
   * @param toFollow The motor controller to follow.
   * @param brakeMode Whether this Talon should be in brake mode or coast mode.
   * @param voltageCompSamples The number of voltage compensation samples to use, or null to not
   *     compensate voltage.
   */
  private static void setMasterForVictor(
      @NotNull VictorSPX victorSPX,
      @NotNull IMotorController toFollow,
      boolean brakeMode,
      @Nullable Integer voltageCompSamples) {
    // Brake mode doesn't automatically follow master
    victorSPX.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);

    // Voltage comp might not follow master either
    if (voltageCompSamples != null) {
      victorSPX.enableVoltageCompensation(true);
      victorSPX.configVoltageCompSaturation(12, 0);
      victorSPX.configVoltageMeasurementFilter(voltageCompSamples, 0);
    } else {
      victorSPX.enableVoltageCompensation(false);
    }

    // Follow the leader
    victorSPX.follow(toFollow);
  }
}
