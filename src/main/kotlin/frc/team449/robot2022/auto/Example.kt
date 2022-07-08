package frc.team449.robot2022.auto
import com.pathplanner.lib.PathPlanner
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.AutoDriveCommand
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.AutoUtils
import frc.team449.control.differential.DifferentialDrive

class Example(private val trajFileName: String, private val maxAcc: Double, private val maxVel: Double, private val drive: DifferentialDrive) {

  fun getRoutine(): AutoRoutine {
    val traj = PathPlanner.loadPath(trajFileName, maxVel, maxAcc)
    val cmd = ParallelCommandGroup(
      AutoDriveCommand.differentialDriveCommand(drive, traj, true),
      AutoUtils.autoSequence(
        listOf(
          1.0 to InstantCommand(),
          2.0 to InstantCommand() // ...
        )
      )
    )
    return AutoRoutine("Example Auto", traj, cmd)
  }
}
