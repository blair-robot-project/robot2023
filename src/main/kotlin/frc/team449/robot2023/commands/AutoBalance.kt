package frc.team449.robot2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2023.auto.AutoConstants
import kotlin.math.PI

class AutoBalance(
  private val controller: PIDController,
  private val drive: SwerveDrive,
  private val speedMetersPerSecond: Double
) : CommandBase() {

  private var timeOfBalance = Double.MAX_VALUE - 2.0

  init {
    addRequirements(drive)
  }

  override fun initialize() {
    addRequirements(drive)
    controller.enableContinuousInput(-PI, PI)
    controller.setTolerance(0.125) /* .05 rad tolerance ~3 degrees */
    controller.setpoint = 0.0
  }
  override fun execute() {
    drive.set(
      ChassisSpeeds(
        controller.calculate(drive.roll.radians) * speedMetersPerSecond,
        0.0,
        0.0
      )
    )

//    if (controller.atSetpoint()) {
//      timeOfBalance = Timer.getFPGATimestamp()
//    }
  }

  override fun isFinished(): Boolean {
    return controller.atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    drive.set(
      ChassisSpeeds(
        0.0,
        0.0,
        0.01
      )
    )
  }

  companion object {
    fun create(drive: SwerveDrive): AutoBalance {
      return AutoBalance(
        PIDController(AutoConstants.AUTO_BAL_KP, 0.0, AutoConstants.AUTO_BAL_KD),
        drive,
        AutoConstants.ADJUST_SPEED
      )
    }
  }
}
