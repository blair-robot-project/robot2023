package frc.team449.robot2023.subsystems

import com.pathplanner.lib.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.RobotBase
import frc.team449.robot2023.commands.PoseAlign

class ControllerBindings(
  private val controller: XboxController,
  private val robot: RobotBase
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      PoseAlign(robot.drive, Pose2d(1.50, 0.0, Rotation2d(-180.00)), maxSpeeds = PathConstraints(1.0, 0.5)).generateCommand()
    )
  }
}
