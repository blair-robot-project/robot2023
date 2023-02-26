package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.robot2023.constants.RobotConstants
import java.util.*
import kotlin.math.PI

object AutoUtil {
  private fun correctTrajectoryListForAlliance(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    if (RobotConstants.ALLIANCE_COLOR == DriverStation.Alliance.Red) {
      for ((index, traj) in trajList.withIndex()) {
        correctedTrajList[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          traj,
          RobotConstants.ALLIANCE_COLOR
        )

        for (s in traj.states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(16.4846 - s.poseMeters.x, 8.02 - s.poseMeters.y, (s.poseMeters.rotation.plus(Rotation2d(PI))))
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
      }
    }

    return correctedTrajList
  }

  private fun transformForFarSide(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    for (traj in trajList) {
      for (s in traj.states) {
        s.poseMeters = Pose2d(s.poseMeters.x, 5.50 - s.poseMeters.y, -s.poseMeters.rotation)
      }
    }

    return correctedTrajList
  }

  fun fullTransform(trajList: MutableList<PathPlannerTrajectory>, farSide: Boolean): MutableList<PathPlannerTrajectory> {
    return if (farSide) {
      transformForFarSide(correctTrajectoryListForAlliance(trajList))
    } else {
      correctTrajectoryListForAlliance(trajList)
    }
  }
}
