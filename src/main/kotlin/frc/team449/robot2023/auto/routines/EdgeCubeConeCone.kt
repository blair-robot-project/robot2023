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

class EdgeCubeConeCone(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to AutoUtil.stowDropCube(robot),
        "stowArm" to AutoUtil.stowAndDeployCone(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.coneHigh },
        "stowCone" to AutoUtil.stowAndDeployCone(robot),
        "highCone" to ArmFollower(robot.arm) { ArmPaths.coneHigh },
        "stopIntake" to AutoUtil.holdIntake(robot),
        "dropCone" to AutoUtil.dropCone(robot),
        "stopConeIntake" to AutoUtil.holdIntake(robot),
        "dropCone2" to AutoUtil.dropCone(robot)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCUBE) {
      Paths.FAR.CUBECONECONE
    } else {
      Paths.WALL.CUBECONECONE
    }
}
