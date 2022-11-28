package frc.team449.robot2022.auto

import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2022.Robot

class Example(
  private val robot: Robot
) {

  fun routine(): AutoRoutine {
    val cmd =
      HolonomicFollower(
        robot.drive,
        Paths.TEST
      )

    return AutoRoutine("Example Auto", cmd)
  }
}
