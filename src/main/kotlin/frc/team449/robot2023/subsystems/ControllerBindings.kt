package frc.team449.robot2023.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import frc.team449.robot2023.subsystems.arm.control.ArmState

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.ZERO_STOW)
    )

    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.STATION_STOW)
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.STOW_STATION)
    )

    JoystickButton(controller, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm, ArmPaths.STOW_MID1)
    )

    JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(
      InstantCommand({ robot.arm.state = ArmState(Rotation2d(), Rotation2d(), 0.0, 0.0) })
    )
  }
}
