package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import frc.team449.robot2023.Robot

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
//    /**  test trajectories */
//    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.ZERO_STOW)
//    )
//
//    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.STOW_ZERO)
//    )
  }
}
