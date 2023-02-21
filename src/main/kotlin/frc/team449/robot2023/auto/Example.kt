package frc.team449.robot2023.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot

class Example(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        yController = PIDController(1.25, 0.0, 0.0),
        drive = robot.drive,
        eventMap = hashMapOf(
          "runIntake" to PrintCommand("woo"),
          "stopIntake" to PrintCommand("woo"),
          "faceTag" to PrintCommand("woo"),
          "shoot" to PrintCommand("woo")
        )
      )

    return routine.fullAuto(Paths.TEST)
  }
}
