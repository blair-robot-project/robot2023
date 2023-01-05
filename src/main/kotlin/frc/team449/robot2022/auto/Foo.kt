package frc.team449.robot2022.auto

import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2022.Robot

class Foo(
  private val robot: Robot
) {

  fun routine(): AutoRoutine {
    val cmd =
      HolonomicFollower(
        robot.drive,
        Paths.foo
      )

    return AutoRoutine("foo", cmd)
  }
}
