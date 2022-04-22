package frc.team449.control.holonomic;

import frc.team449.control.DriveSubsystem;

public interface HolonomicDrive extends DriveSubsystem {
  /**
   * The max speed that this drivetrain can translate at, in meters per second
   */
  double getMaxLinearSpeed();

  /**
   * The max speed that this drivetrain can turn in place at, in radians per second
   */
  double getMaxRotSpeed();
}
