package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot

class Example(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        resetPosition = true,
        drive = robot.drive,
        eventMap = hashMapOf(
          // "printIntake" to PrintCommand("Intaking.... WOAH"),
          // "stopIntake" to PrintCommand("STOPPING INTAKE!")
        ),
        driveEventMap = hashMapOf(
          // 1 to HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
        )
      )

    return routine.constructRoutine(Paths.TEST)
  }
}
