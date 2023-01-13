package frc.team449.robot2022.auto

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.SwerveRoutine
import frc.team449.robot2022.Robot
import frc.team449.robot2022.drive.HeadingAlign

class Example(
  private val robot: Robot
) {

  fun routine(): AutoRoutine {
    val routine =
      SwerveRoutine(
        drive = robot.drive,
        eventMap = hashMapOf(
          "printIntake" to PrintCommand("Intaking.... WOAH"),
          "stopIntake" to PrintCommand("STOPPING INTAKE!!!!!!!!!!")
        ),
        driveEventMap = hashMapOf(
          0 to PrintCommand("IT REACHED THE DRIVE THING"),
          1 to HeadingAlign(robot.drive, Translation2d(), PIDController(1.0, 0.0, 0.0))
        )
      )

    val cmd = routine.constructRoutine(Paths.TEST)

    return AutoRoutine("Example Auto", cmd)
  }
}
