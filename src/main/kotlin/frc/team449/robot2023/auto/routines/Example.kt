package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.Paths

class Example(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        drive = robot.drive,
        eventMap = hashMapOf()
      )

    return routine.fullAuto(Paths.TEST)
  }
}
