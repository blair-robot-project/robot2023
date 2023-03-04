package frc.team449.robot2023.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import kotlin.math.abs

class ControllerBindings(
  private val drivecontroller: XboxController,
  private val mechanismcontroller: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {

    JoystickButton(mechanismcontroller, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.LOW) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kLeftBumper.value).onTrue(
      InstantCommand(robot.intake::pistonOn)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kRightBumper.value).onTrue(
      InstantCommand(robot.intake::pistonRev)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kStart.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.MID) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismcontroller, XboxController.Button.kA.value).onTrue(
      InstantCommand(robot.intake::pistonRev)
        .andThen(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) }
        )
        .withInterruptBehavior(kCancelIncoming)
    )

    POVButton(mechanismcontroller, 0).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }.withInterruptBehavior(kCancelIncoming)
    )

    Trigger { abs(mechanismcontroller.leftY) > 0.3 || abs(mechanismcontroller.rightY) > 0.3 }.whileTrue(
      RepeatCommand(
        InstantCommand(
          {
            val s = robot.arm.state
            s.beta =
              Rotation2d(s.beta.radians - MathUtil.applyDeadband(mechanismcontroller.leftY, .3) * .005)
            s.theta =
              Rotation2d(s.theta.radians - MathUtil.applyDeadband(mechanismcontroller.rightY, .3) * .005)
            robot.arm.state = s
          }
        )
      ).until { abs(mechanismcontroller.leftY) <= 0.3 && abs(mechanismcontroller.rightY) <= 0.3 }
    )

    JoystickButton(drivecontroller, XboxController.Button.kBack.value).onTrue(
      AutoBalance.create(robot.drive)
    )

    JoystickButton(drivecontroller, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )
  }
}
