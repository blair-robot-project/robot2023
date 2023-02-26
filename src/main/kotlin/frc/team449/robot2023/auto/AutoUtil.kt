package frc.team449.robot2023.auto

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.team449.robot2023.constants.RobotConstants
import java.util.*
import java.util.function.BooleanSupplier
import kotlin.math.PI

object AutoUtil {
  fun correctTrajectoryListForAlliance(trajList: MutableList<PathPlannerTrajectory>, isRed: BooleanSupplier): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    if (isRed.asBoolean) {
      for ((index, traj) in correctedTrajList.withIndex()) {
        correctedTrajList[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          traj,
          RobotConstants.ALLIANCE_COLOR
        )

        for (s in correctedTrajList[index].states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(16.4846 - s.poseMeters.x, 8.02 - s.poseMeters.y, (s.poseMeters.rotation.plus(Rotation2d(PI))))
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
      }
    }

    return correctedTrajList
  }

  fun transformForFarSide(trajList: MutableList<PathPlannerTrajectory>): MutableList<PathPlannerTrajectory> {
    val correctedTrajList: MutableList<PathPlannerTrajectory> = MutableList(
      trajList.size
    ) { PathPlannerTrajectory() }

    Collections.copy(correctedTrajList, trajList)

    for ((index, traj) in correctedTrajList.withIndex()) {
      for (s in correctedTrajList[index].states) {
        s.poseMeters = Pose2d(s.poseMeters.x, 5.50 - s.poseMeters.y, -s.poseMeters.rotation)
      }
    }

    return correctedTrajList
  }
}
