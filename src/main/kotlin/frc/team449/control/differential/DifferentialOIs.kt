package frc.team449.control.differential

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import frc.team449.control.OI

/**
 * Helper class to create OIs for a differential drivetrain (arcade, curvature,
 * or tank)
 */
object DifferentialOIs {
  /**
   * Create an OI for arcade drive. One throttle controls forward-backward speed,
   * another controls rotation.
   *
   * @param drive The drivetrain
   * @param xThrottle Throttle to get forward-backward movement
   * @param rotThrottle Throttle to get rotation
   * @param xRamp Used for limiting forward-backward acceleration
   * @param rotRamp Used for limiting rotational acceleration
   */
  fun createArcade(
    drive: DifferentialDrive,
    xThrottle: () -> Double,
    rotThrottle: () -> Double,
    xRamp: SlewRateLimiter,
    rotRamp: SlewRateLimiter
  ): OI = OI {
    scaleAndApplyRamping(
      edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDriveIK(
        xThrottle(),
        rotThrottle(),
        false
      ),
      drive.kinematics,
      drive.maxLinearSpeed,
      xRamp,
      rotRamp
    )
  }

  /**
   * Create OI for curvature drive (drives like a car). One throttle controls
   * forward-backward speed, like arcade, but the other controls curvature instead
   * of rotation. Ramping is still applied to rotation instead of curvature.
   *
   * @param drive The drivetrain
   * @param xThrottle Throttle to get forward-backward movement
   * @param rotThrottle Throttle to get rotation
   * @param xRamp Used for limiting forward-backward acceleration
   * @param rotRamp Used for limiting rotational acceleration
   * @param turnInPlace When this returns true, turns in place instead of turning
   *        like a car
   */
  fun createCurvature(
    drive: DifferentialDrive,
    xThrottle: () -> Double,
    rotThrottle: () -> Double,
    xRamp: SlewRateLimiter,
    rotRamp: SlewRateLimiter,
    turnInPlace: () -> Boolean
  ): OI = OI {
    scaleAndApplyRamping(
      edu.wpi.first.wpilibj.drive.DifferentialDrive.curvatureDriveIK(
        xThrottle(),
        rotThrottle(),
        turnInPlace()
      ),
      drive.kinematics,
      drive.maxLinearSpeed,
      xRamp,
      rotRamp
    )
  }

  /**
   * Create an OI for tank drive. Each throttles controls one side of the drive
   * separately. Each side is also ramped separately.
   *
   * <p>
   * Shame on you if you ever use this.
   *
   * @param drive The drivetrain
   * @param leftThrottle Throttle to get forward-backward movement
   * @param rightThrottle Throttle to get rotation
   * @param leftRamp Used for limiting the left side's acceleration
   * @param rightRamp Used for limiting the right side's acceleration
   */
  fun createTank(
    drive: DifferentialDrive,
    leftThrottle: () -> Double,
    rightThrottle: () -> Double,
    leftRamp: SlewRateLimiter,
    rightRamp: SlewRateLimiter
  ): OI = OI {
    drive.kinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(
        leftRamp.calculate(leftThrottle() * drive.maxLinearSpeed),
        rightRamp.calculate(
          rightThrottle() * drive.maxLinearSpeed
        )
      )
    )
  }

  /**
   * Scales differential drive speeds from [-1, 1] using {@code maxSpeed}, then
   * applies ramping.
   *
   * <p>
   * Do note that although this is given a {@link DifferentialDriveWheelSpeeds}
   * object, the ramping isn't applied to the left and right side but to the
   * linear and rotational velocity using a {@link ChassisSpeeds} object.
   *
   * @param wheelSpeeds The left and right wheel speeds
   * @param kinematics Kinematics object used for turning differential drive wheel
   *        speeds to chassis speeds
   * @param maxSpeed Max linear speed for the drivetrain
   * @param xRamp Used for limiting linear/forward-back acceleration
   * @param rotRamp Used for limiting rotational acceleration
   */
  private fun scaleAndApplyRamping(
    wheelSpeeds: edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds,
    kinematics: DifferentialDriveKinematics,
    maxSpeed: Double,
    xRamp: SlewRateLimiter,
    rotRamp: SlewRateLimiter
  ): ChassisSpeeds {
    val leftVel = wheelSpeeds.left * maxSpeed
    val rightVel = wheelSpeeds.right * maxSpeed
    val chassisSpeeds = kinematics.toChassisSpeeds(
      DifferentialDriveWheelSpeeds(leftVel, rightVel)
    )
    return ChassisSpeeds(
      xRamp.calculate(chassisSpeeds.vxMetersPerSecond),
      0.0,
      rotRamp.calculate(chassisSpeeds.omegaRadiansPerSecond)
    )
  }
}
