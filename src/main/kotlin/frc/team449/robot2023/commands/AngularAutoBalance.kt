package frc.team449.robot2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.auto.AutoConstants
import frc.team449.system.AHRS
import kotlin.math.abs

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
  }

  override fun execute() {
    drive.set(
      ChassisSpeeds(
        speedMetersPerSecond,
        0.0,
        0.0
      )
    )
  }

  override fun isFinished(): Boolean {
    return abs(ahrs.angularXVel()) >= maxAngularVelocity
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
        AutoConstants.MAX_ROT_VEL,
        AutoConstants.ADJUST_SPEED,
        ahrs
      )
    }
  }
}
