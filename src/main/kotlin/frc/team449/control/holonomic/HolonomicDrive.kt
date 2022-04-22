package frc.team449.control.holonomic

import frc.team449.control.DriveSubsystem

interface HolonomicDrive : DriveSubsystem {
  /**
   * The max speed that this drivetrain can translate at, in meters per second
   */
  abstract val maxLinearSpeed: Double

  /**
   * The max speed that this drivetrain can turn in place at, in radians per second
   */
  abstract val maxRotSpeed: Double
}
