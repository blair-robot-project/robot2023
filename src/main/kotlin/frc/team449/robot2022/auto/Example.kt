package frc.team449.robot2022.auto
import com.pathplanner.lib.PathPlanner
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.team449.control.auto.AutoDriveCommand
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.auto.AutoUtils
import frc.team449.control.holonomic.HolonomicDrive

class Example(private val trajFileName: String, private val maxAcc: Double, private val maxVel: Double, private val drive: HolonomicDrive) {

  fun routine(): AutoRoutine {
    val traj = PathPlanner.loadPath(trajFileName, maxVel, maxAcc)
    val controller = HolonomicDriveController(
      PIDController(1.0, .0, .0),
      PIDController(1.0, .0, .0),
      ProfiledPIDController(
        1.5,
        .0,
        .0,
        TrapezoidProfile.Constraints(maxVel, maxAcc)
      )
    )
    val cmd = ParallelCommandGroup(
      // Driving command
      AutoDriveCommand.holonomicDriveCommand(
        drive,
        traj,
        controller,
        true
      ),
      // Doing other stuff commands
      AutoUtils.autoSequence(
        listOf(
          1.0 to InstantCommand(),
          2.0 to InstantCommand() // seconds 0 - 15 to command to execute at that time
        )
      )
    )
    return AutoRoutine("Example Auto", traj, cmd)
  }
}
