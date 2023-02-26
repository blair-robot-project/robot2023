package frc.team449.robot2023.auto

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory

object Paths {
  /** Description of path: Just testing */
  val TEST: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "testing",
      PathConstraints(
        1.0,
        0.25
      )
    )

  /** Description of path: Wall side path that scores a cone, then a cube, and then balances on the station */
  val CONECUBESTATION: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeCubeStation",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone and then balances on the station */
  val CONESTATION: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeStation",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone, then a cube */
  val CONECUBE: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeCube",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone and aligns up to a game piece for teleop */
  val CONE: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallCone",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )
}
