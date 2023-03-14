package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser

class EdgeCube(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to AutoUtil.dropCube(robot),
        "stowArm" to AutoUtil.stowAndDeploy(robot),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
//        "handoff" to robot.groundIntake.handoff()
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCUBE) {
      Paths.FAR.CUBE
    } else {
      Paths.WALL.CUBE
    }
}
