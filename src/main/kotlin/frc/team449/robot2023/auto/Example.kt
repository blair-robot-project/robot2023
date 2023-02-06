package frc.team449.robot2023.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot
import frc.team449.robot2023.commands.HeadingAlign

class Example(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        drive = robot.drive,
        eventMap = hashMapOf(
          "event" to PrintCommand("woah I can run a subsystem command here"),
        ),
        driveEventMap = hashMapOf(
          1 to HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
        )
      )

    return routine.constructRoutine(Paths.TEST)
  }
}
