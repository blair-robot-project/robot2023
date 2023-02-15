package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.commands.HeadingAlign

class ControllerBindings(
  private val controller: XboxController,
  private val robot: frc.team449.robot2023.Robot
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
    )

    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      InstantCommand(robot.shooter::runShooter)
    ).onFalse(
      InstantCommand(robot.shooter::stopShooter)
    )
  }
}
