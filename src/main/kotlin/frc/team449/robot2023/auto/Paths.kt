package frc.team449.robot2023.auto

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

object Paths {
  /** Description of path: Just a test path man */
  val TEST: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "Two Cone",
      PathConstraints(
        1.0,
        0.25
      )
    )
}
