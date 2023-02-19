package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    /**  test trajectories */
//    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.ZERO_STOW)
//    )
//
//    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.STOW_ZERO)
//    )

    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.EXTEND_STOW)
    )

    JoystickButton(controller, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.STOW_EXTEND)
    )

    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      InstantCommand(robot.intake::pistonRev)
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      InstantCommand(robot.intake::pistonOn)
    )
  }
}
