package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil

class DropCone(
  private val robot: Robot
) : RoutineStructure {

  override val routine = HolonomicRoutine(
    drive = robot.drive,
    eventMap = hashMapOf()
  )

  override val trajectory = mutableListOf(PathPlannerTrajectory())

  override fun createCommand(): Command {
    return AutoUtil.dropCone(robot)
  }
}
