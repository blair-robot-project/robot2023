package frc.team449.robot2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem

class HeadingAlign(
  private val drive: DriveSubsystem,
  private val point: Translation2d,
  private val headingPID: PIDController
) : CommandBase() {

  init {
    addRequirements(drive)
    headingPID.enableContinuousInput(-Math.PI, Math.PI)
    headingPID.setTolerance(0.05)
  }

  private var fieldToRobot = Translation2d()
  private var robotToPoint = Translation2d()

  override fun execute() {
    fieldToRobot = drive.pose.translation
    robotToPoint = point - fieldToRobot
    headingPID.setpoint = robotToPoint.angle.radians

    drive.set(
      ChassisSpeeds(
        0.0,
        0.0,
        headingPID.calculate(drive.heading.radians)
      )
    )
  }

  override fun isFinished(): Boolean {
    return headingPID.atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}