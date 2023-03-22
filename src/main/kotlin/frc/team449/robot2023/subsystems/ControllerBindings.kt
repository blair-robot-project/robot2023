package frc.team449.robot2023.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.ArmSweep
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import kotlin.math.abs

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  fun bindButtons() {

    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      InstantCommand(robot.endEffector::intake)
    ).onFalse(
      InstantCommand(robot.endEffector::stop).andThen(
        InstantCommand(robot.endEffector::holdIntake)
      )
    )

    JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
      InstantCommand(robot.endEffector::intakeReverse)
    ).onFalse(
      InstantCommand(robot.endEffector::stop)
    )

    // drive speed overdrive trigger
    Trigger { driveController.rightTriggerAxis >= .8 }.onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 2.0 })
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
    )

    JoystickButton(mechanismController, XboxController.Button.kRightBumper.value).onTrue(
      InstantCommand(robot.endEffector::pistonOn).andThen(
        ConditionalCommand(
          InstantCommand({ robot.arm.desiredState = ArmConstants.CUBE }),
          InstantCommand()
        ) { robot.arm.desiredState == ArmConstants.CONE }
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value).onTrue(
      InstantCommand(robot.endEffector::pistonRev).andThen(
        ConditionalCommand(
          InstantCommand({ robot.arm.desiredState = ArmConstants.CONE }),
          InstantCommand()
        ) { robot.arm.desiredState == ArmConstants.CUBE }
      )
    )

    JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.SINGLE) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kBack.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.DOUBLE) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kStart.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }.withInterruptBehavior(kCancelIncoming)
        .andThen(
          ConditionalCommand(
            InstantCommand({ robot.arm.state = ArmConstants.CUBE }),
            InstantCommand({ robot.arm.state = ArmConstants.CONE })
          ) { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kForward }
        )
    )

    JoystickButton(mechanismController, XboxController.Button.kX.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.MID) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kY.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.withInterruptBehavior(kCancelIncoming)
    )

    Trigger { abs(mechanismController.rightTriggerAxis) > 0.1 }.onTrue(
      ArmSweep(
        robot.arm,
        { mechanismController.rightTriggerAxis },
        Rotation2d.fromDegrees(15.0)
      ).until { abs(mechanismController.rightTriggerAxis) < 0.1 }
    )

    Trigger { abs(mechanismController.leftY) > 0.3 || abs(mechanismController.rightY) > 0.3 }.onTrue(
      RepeatCommand(
        InstantCommand(
          {
            val newState = robot.arm.desiredState.copy()
            newState.beta =
              Rotation2d(newState.beta.radians - MathUtil.applyDeadband(mechanismController.leftY, .3) * .005)
            newState.theta =
              Rotation2d(newState.theta.radians - MathUtil.applyDeadband(mechanismController.rightY, .3) * .005)
            robot.arm.state = newState
          }
        )
      ).until { abs(mechanismController.leftY) <= 0.3 && abs(mechanismController.rightY) <= 0.3 }
    )

    JoystickButton(driveController, XboxController.Button.kBack.value).onTrue(
      AutoBalance.create(robot.drive)
    )

    JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )
  }
}
