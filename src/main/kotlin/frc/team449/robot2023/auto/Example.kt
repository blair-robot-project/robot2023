package frc.team449.robot2023.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.HeadingAlign

class Example(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        yController = PIDController(1.25, 0.0, 0.0),
        drive = robot.drive,
        eventMap = hashMapOf(
          "runIntake" to InstantCommand(robot.intake::runIntakeForward),
          "stopIntake" to InstantCommand(robot.intake::stopIntake),
          "faceTag" to HeadingAlign(robot.drive, Translation2d()),
          "shoot" to InstantCommand(robot.shooter::runShooter)
        )
      )

    return routine.fullAuto(Paths.TEST).andThen(
      InstantCommand(
        robot.shooter::stopShooter
      )
    )
  }
}
