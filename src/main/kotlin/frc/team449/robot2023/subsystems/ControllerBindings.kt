package frc.team449.robot2023.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.control.TurnCommand
import frc.team449.robot2023.Robot
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
      InstantCommand(robot.intake::pistonRev)
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      InstantCommand(robot.intake::pistonOn)
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
