package frc.team449.robot2022.auto

import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2022.Robot

class Example(
  private val robot: Robot
) {

  fun routine(): AutoRoutine {
    val routine =
      HolonomicRoutine(
        drive = robot.drive,
        eventMap = hashMapOf(
          "printIntake" to PrintCommand("Intaking.... WOAH"),
          "printStopIntake" to PrintCommand("STOPPING INTAKE!!!!!!!!!!")
        ),
        driveEventMap = hashMapOf(
//          1 to HeadingAlign(
//            robot.drive,
//            Translation2d(),
//            PIDController(1.0, 0.0, 0.0)
//          )
        )
      )

    val cmd = routine.fullAuto(Paths.TEST)

    return AutoRoutine("Example Auto", cmd)
  }
}
