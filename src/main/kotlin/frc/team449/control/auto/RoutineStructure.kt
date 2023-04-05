package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.robot2023.Robot

interface RoutineStructure {

  val routine: HolonomicRoutine

  val trajectory: MutableList<PathPlannerTrajectory>

  fun createCommand(robot: Robot): Command {
    return routine.createRoutine(trajectory).alongWith(
      InstantCommand(robot.arm::holdArm)
    )
  }
}
