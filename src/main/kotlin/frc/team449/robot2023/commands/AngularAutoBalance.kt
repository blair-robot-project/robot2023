package frc.team449.robot2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.auto.AutoConstants
import frc.team449.system.AHRS

class AngularAutoBalance(
  private val drive: SwerveDrive,
  private val maxAngularVelocity: Double,
  private val speedMetersPerSecond: Double,
  private val ahrs: AHRS
) : CommandBase() {

  init {
    addRequirements(drive)
  }

  override fun initialize() {
    addRequirements(drive)
  }

  override fun execute() {
    drive.desiredSpeeds = ChassisSpeeds(speedMetersPerSecond, 0.0, 0.0)
  }

  override fun isFinished(): Boolean {
    return ahrs.angularVel >= maxAngularVelocity
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }

  companion object {
    fun create(
      drive: SwerveDrive,
      ahrs: AHRS
    ): AngularAutoBalance {
      return AngularAutoBalance(
        drive,
        AutoConstants.ADJUST_SPEED,
        AutoConstants.MAX_ROT_VEL,
        ahrs
      )
    }
  }
}
