package frc.team449.robot2023.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.subsystems.arm.ArmState

class ControllerBindings(
  private val controller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    JoystickButton(controller, XboxController.Button.kA.value).onTrue(
      InstantCommand({
        robot.arm.state = ArmState(
          Rotation2d(),
          Rotation2d(),
          0.0,
          0.0
        )
      }
      )
    )

    JoystickButton(controller, XboxController.Button.kX.value).onTrue(
      InstantCommand({
        robot.arm.state = ArmState(
          Rotation2d.fromDegrees(10.0),
          Rotation2d.fromDegrees(10.0),
          0.0,
          0.0
        )
      }
      )
    )

    JoystickButton(controller, XboxController.Button.kB.value).onTrue(
      InstantCommand({
        robot.arm.state = ArmState(
          Rotation2d.fromDegrees(45.0),
          Rotation2d.fromDegrees(45.0),
          0.0,
          0.0
        )
      }
      )
    )

    JoystickButton(controller, XboxController.Button.kY.value).onTrue(
      InstantCommand({
        robot.arm.state = ArmState(
          Rotation2d.fromDegrees(90.0),
          Rotation2d.fromDegrees(-45.0),
          0.0,
          0.0
        )
      }
      )
    )
  }
}
