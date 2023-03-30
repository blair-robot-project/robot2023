package frc.team449.robot2023.commands.driveAlign

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem

/**
 * @param drive The holonomic drive you want to align with
 * @param point The point in 2d space you want the drivetrain to face towards
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 */
class HeadingAlign(
  private val drive: DriveSubsystem,
  private val point: Translation2d,
  private val headingPID: PIDController = PIDController(1.5, 0.0, 0.0)
) : CommandBase() {

  init {
    addRequirements(drive)
    headingPID.enableContinuousInput(-Math.PI, Math.PI)
    headingPID.setTolerance(0.075)
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
