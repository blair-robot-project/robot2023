package frc.team449.robot2022.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem

class HeadingAlign(
  val drive: DriveSubsystem,
  val point: Translation2d,
  val headingPID: PIDController
) : CommandBase() {
  override fun initialize() {
    val fieldToRobot = drive.pose.translation
    val robotToPoint = point - fieldToRobot
    headingPID.setpoint = robotToPoint.angle.radians
  }

  override fun execute() {
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
