package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.RobotBase
import frc.team449.robot2023.commands.HeadingAlign

class ControllerBindings(
  private val controller: XboxController,
  private val robot: RobotBase,
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
    )
    JoystickButton(controller, XboxController.Button.kA.value).toggleOnTrue(
      InstantCommand(robot.intake::run)
    ).toggleOnFalse(
      InstantCommand(robot.intake::stop)
    )
    JoystickButton(controller, XboxController.Button.kB.value).toggleOnTrue(
      InstantCommand(robot.intake::runReverse)
    ).toggleOnFalse(
      InstantCommand(robot.intake::stop)
    )
  }
}
