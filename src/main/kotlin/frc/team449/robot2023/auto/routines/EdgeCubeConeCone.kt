package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
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
        "dropCube" to AutoUtil.dropCube(robot).andThen(PrintCommand("FSIOFSFHODUIOASHFBIDSOAUFHI")),
        "stowArm" to AutoUtil.stowAndDeployCone(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.stowHigh },
        "stowCone" to AutoUtil.stowAndDeployCube(robot),
        "midCone" to ArmFollower(robot.arm) { ArmPaths.stowMid },
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCone" to SequentialCommandGroup(
          RepeatCommand(
            InstantCommand(
              {
                val startState = robot.arm.desiredState.copy()
                robot.arm.desiredState.beta = startState.beta + Rotation2d.fromDegrees(0.175)
              }
            )
          ).withTimeout(0.5),
          InstantCommand(robot.endEffector::pistonRev)
        ),
        "stopConeIntake" to AutoUtil.retractGroundIntake(robot),
        "dropCone2" to SequentialCommandGroup(
          RepeatCommand(
            InstantCommand(
              {
                val startState = robot.arm.desiredState.copy()
                robot.arm.desiredState.beta = startState.beta + Rotation2d.fromDegrees(0.175)
              }
            )
          ).withTimeout(0.5),
          InstantCommand(robot.endEffector::pistonRev)
        )
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCUBE) {
      Paths.FAR.CUBECONECONE
    } else {
      Paths.WALL.CUBECONECONE
    }
}
