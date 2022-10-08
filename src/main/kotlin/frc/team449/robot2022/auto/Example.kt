package frc.team449.robot2022.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.AutoUtils
import frc.team449.control.auto.HolonomicFollower
import frc.team449.robot2022.RobotContainer2022

class Example(
  private val robot: RobotContainer2022
) {

  fun routine(): AutoRoutine {
    val traj = Paths.FIVE_BALL

    val cmd = ParallelCommandGroup(
      HolonomicFollower(
        robot.drive,
        traj,
        true,
        AutoConstants.MAX_ROTVEL,
        AutoConstants.MAX_ROTACC,
        0.02,
        0.01,
        1.0
      ),
      // Doing other stuff commands
      AutoUtils.autoSequence(
        listOf(
          1.0 to InstantCommand(),
          2.0 to InstantCommand() // seconds 0 - 15 to command to execute at that time
        )
      )
    )
    return AutoRoutine("Example Auto", cmd)
  }
}
