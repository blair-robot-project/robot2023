package frc.team449.robot2023.auto

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

object Paths {
  /** Description of path: Just a test path man */
  val TEST: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeCubeStation",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )
}
