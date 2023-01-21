package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.HeadingAlign
import frc.team449.robot2023.subsystems.arm.CartesianArmState

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
    )
    JoystickButton(controller, 1).onTrue(
      InstantCommand({
        robot.arm.coordinate =
          CartesianArmState(
            1.1, 1.02, 1.0, 0.0
          )
      }
      )
    )
  }
}
