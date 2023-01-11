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
          "marker1" to PrintCommand("Lower Intake Command"),
          "marker2" to PrintCommand("Intake Command"),
          "marker3" to PrintCommand("Win Game Command")
        ),
        driveEventMap = hashMapOf(
          1 to PrintCommand("Align Drive Command"),
          2 to PrintCommand("Smash into Alliance Wall Command"),
          3 to PrintCommand("Commands that require drive")
        )
      )

    val cmd = routine.fullAuto(Paths.TEST)

    return AutoRoutine("Example Auto", cmd)
  }
}
