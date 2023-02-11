package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.ZERO_STOW)
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.STOW_ZERO)
    )
  }
}
