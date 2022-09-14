package frc.team449.robot2022.auto

import com.pathplanner.lib.PathPlanner
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.*
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants.ROT_CONTROLLER
import frc.team449.robot2022.auto.AutoConstants.xController
import frc.team449.robot2022.auto.AutoConstants.yController

class Example(
  private val trajFileName: String,
  private val drive: HolonomicDrive
) {

  fun routine(): AutoRoutine {
    val traj = PathPlanner.loadPath(trajFileName, AutoConstants.MAX_VEL, AutoConstants.MAX_ACC)

    val cmd = ParallelCommandGroup(
      InstantCommand({ // reset before driving
        xController.reset()
        yController.reset()
        ROT_CONTROLLER.reset(traj.initialPose.rotation.radians)
      }).andThen(
        HolonomicFollower(
          drive,
          SwerveTrajectory(AutoConstants.points),
          true
        )
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
