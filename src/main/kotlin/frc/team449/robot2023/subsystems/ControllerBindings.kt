package frc.team449.robot2023.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelIncoming
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.arm.ArmSweep
import frc.team449.robot2023.commands.autoBalance.AutoBalance
import frc.team449.robot2023.commands.driveAlign.DoubleAlign
import frc.team449.robot2023.constants.RobotConstants
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import kotlin.math.abs

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  private fun changeCone(): Command {
    return robot.endEffector.runOnce(robot.endEffector::pistonOn).andThen(
      ConditionalCommand(
        SequentialCommandGroup(
          robot.groundIntake.retract(),
          WaitCommand(0.5), // wait for the intake to fully retract (this might take more time)
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
        ),
        InstantCommand()
      ) { robot.arm.desiredState == ArmConstants.CUBE }
    )
  }

  private fun changeCube(): Command {
    return robot.endEffector.runOnce(robot.endEffector::pistonRev).andThen(
      ConditionalCommand(
        SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) },
          WaitUntilCommand {
            robot.arm.distanceBetweenStates(robot.arm.state, ArmConstants.CUBE) <= 0.05 &&
              robot.arm.state.betaVel <= 0.05
            robot.arm.state.thetaVel <= 0.05
          },
          robot.groundIntake.deploy()
        ),
        InstantCommand()
      ) { robot.arm.desiredState == ArmConstants.CONE }
    )
  }

  fun bindButtons() {
    // TODO: LED stuff if we get there
//    Trigger { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kReverse }.onTrue(
//      CubeAnimation(robot.light)
//    ).onFalse(
//      ConeAnimation(robot.light)
//    )
//
//    Trigger { !robot.infrared.get() }.onTrue(
//      PickupBlink().blinkGreen(robot)
//    )

    JoystickButton(driveController, XboxController.Button.kRightBumper.value).onTrue(
      ConditionalCommand(
        robot.endEffector.runOnce(robot.endEffector::intake),
        SequentialCommandGroup(
          robot.groundIntake.teleopCube(),
          robot.endEffector.runOnce(robot.endEffector::intake)
        )
      ) { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kForward }
    ).onFalse(
      SequentialCommandGroup(
        robot.groundIntake.runOnce(robot.groundIntake::stop),
        robot.endEffector.runOnce(robot.endEffector::holdIntake)
      )
    )

    Trigger { driveController.rightTriggerAxis > 0.8 }.onTrue(
      ConditionalCommand(
        SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) },
          WaitUntilCommand {
            robot.arm.distanceBetweenStates(robot.arm.state, ArmConstants.CUBE) <= 0.0175 &&
              robot.arm.state.betaVel <= 0.025 &&
              robot.arm.state.thetaVel <= 0.025
          },
          robot.groundIntake.deploy(),
          robot.groundIntake.teleopCube()
        ).alongWith(
          RepeatCommand(InstantCommand(robot.arm::holdArm))
        ),
        SequentialCommandGroup(
          robot.groundIntake.deploy(),
          robot.groundIntake.intakeCone()
        )
      ) { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kReverse }
    ).onFalse(
      SequentialCommandGroup(
        robot.groundIntake.retract(),
        robot.groundIntake.runOnce(robot.groundIntake::stop),
        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
      )
    )

    JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
      robot.endEffector.runOnce(robot.endEffector::intakeReverse).andThen(
        robot.groundIntake.outtake()
      )
    ).onFalse(
      robot.endEffector.runOnce(robot.endEffector::stop).andThen(
        robot.groundIntake.runOnce(robot.groundIntake::stop)
      )
    )

    // drive speed overdrive trigger
    Trigger { driveController.rightTriggerAxis >= .8 }.onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 2.0 })
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
    )

    JoystickButton(mechanismController, XboxController.Button.kRightBumper.value).onTrue(
      changeCube()
    )

    JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value).onTrue(
      changeCone()
    )

    Trigger { robot.arm.desiredState == ArmConstants.STOW }.onTrue(
      robot.endEffector.runOnce(robot.endEffector::strongHoldIntake)
    ).onFalse(
      robot.endEffector.runOnce(robot.endEffector::holdIntake)
    )

    JoystickButton(mechanismController, XboxController.Button.kB.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.SINGLE) }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kBack.value).onTrue(
      ArmFollower(robot.arm) {
        robot.arm.chooseTraj(ArmConstants.STOW)
      }.withInterruptBehavior(kCancelIncoming)
    )

    JoystickButton(mechanismController, XboxController.Button.kStart.value).onTrue(
      ArmFollower(robot.arm) {
        robot.arm.chooseTraj(ArmConstants.BACK)
      }.withInterruptBehavior(kCancelIncoming)
    )

//    JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
//      ConditionalCommand(
//        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CUBE) }.withInterruptBehavior(kCancelIncoming),
//        ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }.withInterruptBehavior(kCancelIncoming)
//      ) { robot.endEffector.chooserPiston.get() == DoubleSolenoid.Value.kReverse }
//    )

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

//    TODO: Orbit Heading Align
//    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
//      HeadingAlign(
//        robot.drive,
//        robot.oi,
//        Translation2d(),
//      ).until { driveController.aButtonReleased }
//    )

    Trigger { abs(mechanismController.leftY) > 0.3 || abs(mechanismController.rightY) > 0.3 }.onTrue(
      RepeatCommand(
        InstantCommand(
          {
            val newState = robot.arm.desiredState.copy()
            newState.beta =
              Rotation2d(newState.beta.radians - MathUtil.applyDeadband(mechanismController.leftY, .3) * .005)
            newState.theta =
              Rotation2d(newState.theta.radians - MathUtil.applyDeadband(mechanismController.rightY, .3) * .005)
            robot.arm.moveToState(newState)
          }
        )
      ).until { abs(mechanismController.leftY) <= 0.3 && abs(mechanismController.rightY) <= 0.3 }
    )

    JoystickButton(mechanismController, XboxController.Button.kA.value).onTrue(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.DOUBLE) }
        .andThen(InstantCommand(robot.endEffector::intake))
        .andThen(InstantCommand(robot.endEffector::pistonOn))
    ).onFalse(
      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.BACK) }
    )

//    JoystickButton(driveController, XboxController.Button.kA.value).onTrue(
//      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.SINGLE) }.withInterruptBehavior(kCancelIncoming)
//    )

//    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
//      changeCube()
//    )
//
//    JoystickButton(driveController, XboxController.Button.kX.value).onTrue(
//      changeCone()
//    )

//    JoystickButton(driveController, XboxController.Button.kY.value).onTrue(
//      ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.withInterruptBehavior(kCancelIncoming)
//    )

    JoystickButton(driveController, XboxController.Button.kBack.value).onTrue(
      AutoBalance.create(robot.drive)
    )

    JoystickButton(driveController, XboxController.Button.kStart.value).onTrue(
      InstantCommand({ robot.drive.heading = Rotation2d(0.0) })
    )

    JoystickButton(driveController, XboxController.Button.kB.value).onTrue(
      InstantCommand({ DoubleAlign().rightDoubleAlign(robot, driveController).schedule() })
    )

    JoystickButton(driveController, XboxController.Button.kX.value).onTrue(
      InstantCommand({ DoubleAlign().leftDoubleAlign(robot, driveController).schedule() })
    )
  }
}
