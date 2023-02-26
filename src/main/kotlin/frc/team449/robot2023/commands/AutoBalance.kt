package frc.team449.robot2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.holonomic.SwerveDrive
import kotlin.math.PI

class AutoBalance(
  private val controller: PIDController,
  private val drive: SwerveDrive,
  private val speedMetersPerSecond: Double
) : CommandBase() {

  override fun initialize() {
    controller.enableContinuousInput(-PI, PI)
    controller.setTolerance(0.05) // .05 rad tolerance ~3 degrees
  }
  override fun execute() {
    drive.set(
      ChassisSpeeds(
        controller.calculate(drive.roll.radians) * speedMetersPerSecond,
        0.0,
        0.0
      )
    )
  }

  override fun isFinished(): Boolean {
    return controller.atSetpoint()
  }
}
