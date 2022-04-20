package frc.team449.system;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class AHRS {
  private final com.kauailabs.navx.frc.AHRS navx;
  private double headingOffset;

  public AHRS(SerialPort.Port port) {
    this.navx = new com.kauailabs.navx.frc.AHRS(port);
  }

  public void setHeading(Rotation2d heading) {
    this.headingOffset = heading.getDegrees() - this.navx.getFusedHeading();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(headingOffset + this.navx.getFusedHeading());
  }
}
