package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.ProfiledPoseAlign
import frc.team449.robot2023.commands.TrajectoryPoseAlign
import kotlin.math.PI

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismsController: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
      ProfiledPoseAlign(
        robot.drive,
        Pose2d(1.0, 0.0, Rotation2d(PI / 2)),
        ProfiledPIDController(
          1.0,
          0.0,
          0.0,
          TrapezoidProfile.Constraints(1.0, 1.0)
        ),
        ProfiledPIDController(
          1.0,
          0.0,
          0.0,
          TrapezoidProfile.Constraints(1.0, 1.0)
        ),
        PIDController(
          1.0,
          0.0,
          0.0
        )
      )
    )

    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
      TrajectoryPoseAlign(
        robot.drive,
        Pose2d(1.0, 0.0, Rotation2d(PI / 2))
      ).generateCommand()
    )
  }
}
