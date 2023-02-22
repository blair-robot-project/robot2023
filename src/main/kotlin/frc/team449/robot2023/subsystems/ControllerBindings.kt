package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.control.TurnCommand
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.arm.ArmConstants
import kotlin.math.PI

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

    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      robot.arm.chooseTraj(ArmConstants.MID)
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      robot.arm.chooseTraj(ArmConstants.CONE)
    )

    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      robot.arm.chooseTraj(ArmConstants.STOW)
    )

    JoystickButton(controller, XboxController.Button.kY.value).onTrue(
      robot.arm.chooseTraj(ArmConstants.HIGH)
    )

    JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(
      TurnCommand(
        robot.drive,
        robot.oi,
        PIDController(0.1, 0.0, 0.0),
        PI / 2
      )
    )

    JoystickButton(controller, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )
  }
}
