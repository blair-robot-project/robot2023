package frc.team449.robot2022.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.AutoUtils
import frc.team449.control.holonomic.SwerveDrive

class Pose2dAuto(private val drive: SwerveDrive, private val maxAcc: Double, private val maxVel: Double) {
  fun routine(): AutoRoutine {
    val swerveKinematics = drive.getKinematics()
    val trajConfig = TrajectoryConfig(maxVel, maxAcc).setKinematics(swerveKinematics)
    val traj = TrajectoryGenerator.generateTrajectory(
      Pose2d(5.0, 5.0, Rotation2d(0.0)),
      listOf(
        Translation2d(4.0, 3.0),
        Translation2d(6.0, 6.0)
      ),
      Pose2d(8.0, 3.0, Rotation2d.fromDegrees(180.0)),
      trajConfig
    )
    val xController = PIDController(1.0, 0.0, 0.0)
    val yController = PIDController(1.0, 0.0, 0.0)
    val thetaController = ProfiledPIDController(1.5, .0, .0, TrapezoidProfile.Constraints(maxVel, maxAcc))
    thetaController.enableContinuousInput(-Math.PI, Math.PI)

    val cmd = ParallelCommandGroup(
      SequentialCommandGroup(
        InstantCommand({ drive.resetOdometry(traj.initialPose) }),
        SwerveControllerCommand(
          traj,
          drive::pose,
          swerveKinematics,
          xController,
          yController,
          thetaController,
          { desiredStates: Array<SwerveModuleState> ->
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxVel)
            drive.set(swerveKinematics.toChassisSpeeds(*desiredStates))
          },
          drive
        ),
        InstantCommand(drive::stop)
      ),
      // Doing other stuff commands
      AutoUtils.autoSequence(
        listOf(
          1.0 to InstantCommand(),
          2.0 to InstantCommand() // seconds 0 - 15 to command to execute at that time
        )
      )
    )
    return AutoRoutine("WPIlib Auto", traj, cmd)
  }
}
