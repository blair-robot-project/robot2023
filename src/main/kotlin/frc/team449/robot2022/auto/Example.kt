package frc.team449.robot2022.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.AutoUtils
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2022.Robot

class Example(
  private val robot: Robot
) {

  fun routine(): AutoRoutine {
    val traj = Paths.FIVE_BALL

    val cmd = ParallelCommandGroup(
      HolonomicFollower(
        robot.drive,
        traj,
        true,
        AutoConstants.MAX_ROTVEL,
        AutoConstants.MAX_ROTACC
      ),
      // Doing other stuff commands
      AutoUtils.autoSequence(
        listOf(
          1.0 to InstantCommand() // time(s) to command
        )
      )
    )
    return AutoRoutine("Example Auto", cmd)
  }
}
