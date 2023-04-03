package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeCubeCone(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to AutoUtil.stowDropCube(robot),
        "stowArm" to AutoUtil.deployCone(robot),
        "stopIntake" to AutoUtil.holdIntake(robot),
        "dropCone" to ArmFollower(robot.arm) { ArmPaths.coneHigh }.andThen(AutoUtil.dropCone(robot)),
        "retractArm" to ArmFollower(robot.arm) { ArmPaths.highStow },
        "stopIntake2" to AutoUtil.retractGroundIntake(robot),
        "stowCone" to AutoUtil.deployCone(robot)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCUBE) {
      Paths.FAR.CUBECONE
    } else {
      Paths.WALL.CUBECONE
    }
}
