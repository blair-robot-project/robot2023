package frc.team449.robot2023.commands.driveAlign

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.control.DriveSubsystem
import frc.team449.control.OI
import frc.team449.robot2023.auto.AutoConstants
import frc.team449.robot2023.constants.RobotConstants

/**
 * @param drive The holonomic drive you want to align with
 * @param point The point in 2d space you want the drivetrain to face towards
 * @param headingPID The non-Profiled PID controller you want to use for fixing rotational error
 */
class HeadingAlign(
  private val drive: DriveSubsystem,
  private val oi: OI,
  private val point: Translation2d,
  private val headingPID: ProfiledPIDController = ProfiledPIDController(
    AutoConstants.DEFAULT_ROTATION_KP,
    0.0,
    0.0,
    TrapezoidProfile.Constraints(RobotConstants.MAX_ROT_SPEED, RobotConstants.RATE_LIMIT)
  )
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
    headingPID.goal = TrapezoidProfile.State(robotToPoint.angle.radians, 0.0)

    drive.set(
      ChassisSpeeds(
        oi.get().vxMetersPerSecond,
        oi.get().vyMetersPerSecond,
        headingPID.calculate(drive.heading.radians)
      )
    )
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
