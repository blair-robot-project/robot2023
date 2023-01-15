package frc.team449.robot2022.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.RobotBase
import frc.team449.robot2022.commands.HeadingAlign

class ControllerBindings(
  private val controller: XboxController,
  private val robot: RobotBase
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
    )
  }
}
