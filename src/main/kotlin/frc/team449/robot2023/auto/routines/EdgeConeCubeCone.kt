package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCubeCone(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCube" to InstantCommand(robot.endEffector::autoReverse),
        "stowArm" to AutoUtil.stowAndDeployCube(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.cubeHigh },
        "stowCube" to AutoUtil.stowAndDeployCube(robot),
        "midCube" to ArmFollower(robot.arm) { ArmPaths.coneHigh },
        "stopIntake" to AutoUtil.holdIntake(robot),
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stopCubeIntake" to AutoUtil.holdIntake(robot),
        "dropCube2" to InstantCommand(robot.endEffector::autoReverse)
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCONE) {
      Paths.FAR.CONECUBECONE
    } else {
      Paths.WALL.CONECUBECONE
    }
}
