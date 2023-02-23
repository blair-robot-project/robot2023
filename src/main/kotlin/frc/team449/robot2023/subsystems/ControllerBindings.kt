package frc.team449.robot2023.subsystems

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class ControllerBindings(
  private val drivecontroller: XboxController,
  private val mechanismcontroller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {
    /**  test trajectories */
//    JoystickButton(drivecontroller, XboxController.Button.kA.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.ZERO_STOW)
//    )
//
//    JoystickButton(drivecontroller, XboxController.Button.kB.value).onTrue(
//      ArmFollower(robot.arm, ArmPaths.STOW_ZERO)
//    )

    JoystickButton(mechanismcontroller, XboxController.Button.kA.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.LOW) }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.MID) }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kLeftBumper.value).onTrue(
      InstantCommand(robot.intake::pistonRev)
        .andThen(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )

    JoystickButton(drivecontroller, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )
  }
}
