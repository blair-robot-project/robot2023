package frc.team449.system.encoder;

public final class EncoderConfig {
  public final double gearing;
  public final double unitPerRotation;
  public final int encoderCPR;

  /**
   * Configuration for an encoder
   *
   * @param gearing The factor to multiply output by after the encoder measurement, e.g. this would
   *     be 70 if there is a 70:1 gearing after the encoder
   * @param unitPerRotation How many units it travels per rotation, e.g. this would be about 2pi*r
   *     for a wheel
   * @param encoderCPR Counts per rotation of the encoder
   */
  public EncoderConfig(double gearing, double unitPerRotation, int encoderCPR) {
    this.gearing = gearing;
    this.unitPerRotation = unitPerRotation;
    this.encoderCPR = encoderCPR;
  }

  /** Convert encoder units to meters or another unit we want */
  public double encoderToUnit(double revs) {
    return revs * unitPerRotation * gearing / encoderCPR;
  }

  public double unitToEncoder(double units) {
    return units * encoderCPR / unitPerRotation / gearing;
  }
}
