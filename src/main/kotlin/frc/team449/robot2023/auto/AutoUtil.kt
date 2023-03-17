package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.team449.robot2023.Robot
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower
import java.util.*
import java.util.function.BooleanSupplier
import kotlin.math.PI

object AutoUtil {
  fun transformForFarSide(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    for ((index, _) in correctedTrajList.withIndex()) {
      for (s in correctedTrajList[index].states) {
        s.poseMeters = Pose2d(s.poseMeters.x, 5.50 - s.poseMeters.y, -s.poseMeters.rotation)
      }
    }

    return correctedTrajList
  }

  fun transformForAlliance(pathGroup: MutableList<PathPlannerTrajectory>, isRed: BooleanSupplier): MutableList<PathPlannerTrajectory> {
    val correctedPathGroup: MutableList<PathPlannerTrajectory> = MutableList(
      pathGroup.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedPathGroup, pathGroup)

    if (isRed.asBoolean) {
      for ((index, _) in correctedPathGroup.withIndex()) {
        correctedPathGroup[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          correctedPathGroup[index],
          DriverStation.getAlliance()
        )

        for (s in correctedPathGroup[index].states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(16.4846 - s.poseMeters.x, 8.02 - s.poseMeters.y, (s.poseMeters.rotation.plus(Rotation2d(PI))))
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
      }
    }

    return correctedPathGroup
  }

  fun dropCone(robot: Robot): Command {
    return SequentialCommandGroup(
      ArmFollower(robot.arm) { ArmPaths.STOW_HIGH },
      WaitCommand(1.75),
//      RepeatCommand(InstantCommand(
//        {
//          val startState = robot.arm.desiredState.copy()
//          robot.arm.desiredState.beta = startState.beta + Rotation2d.fromDegrees(0.135)
//        }
//      )).withTimeout(2.25),
      InstantCommand(robot.endEffector::intakeReverse)
    )
  }

  fun dropCube(robot: Robot): Command {
    return SequentialCommandGroup(
      ArmFollower(robot.arm) { ArmPaths.STOW_HIGH },
      WaitCommand(0.5),
      InstantCommand(robot.endEffector::intakeReverse)
    )
  }

  fun stowAndDeployCube(robot: Robot): Command {
    return SequentialCommandGroup(
      ArmFollower(robot.arm) { ArmPaths.HIGH_STOW },
      InstantCommand({ robot.arm.desiredState = ArmConstants.INTAKE }),
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonRev),
    )
  }

  fun stowAndDeployCone(robot: Robot): Command {
    return SequentialCommandGroup(
      ArmFollower(robot.arm) { ArmPaths.HIGH_STOW },
      InstantCommand({ robot.arm.desiredState = ArmConstants.INTAKE }),
      InstantCommand(robot.endEffector::intake),
      InstantCommand(robot.endEffector::pistonOn),
    )
  }

  fun retractGroundIntake(robot: Robot): Command {
    return InstantCommand(robot.endEffector::stop).andThen(
      InstantCommand({ robot.arm.desiredState = ArmConstants.STOW })
    )
  }
}
