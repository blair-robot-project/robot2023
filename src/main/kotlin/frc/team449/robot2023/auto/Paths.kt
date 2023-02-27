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
  val WALLCONECUBESTATION: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeCubeStation",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone and then balances on the station */
  val WALLCONESTATION: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeStation",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone, then a cube */
  val WALLCONECUBE: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallConeCube",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Wall side path that scores a cone and aligns up to a game piece for teleop */
  val WALLCONE: MutableList<PathPlannerTrajectory> =
    PathPlanner.loadPathGroup(
      "wallCone",
      PathConstraints(
        2.5,
        0.75 // Once worked out, at least 3.5
      )
    )

  /** Description of path: Far (near center of field) side path that scores a cone, then a cube, and then balances on the station */
  val FARCONECUBESTATION: MutableList<PathPlannerTrajectory> =
    AutoUtil.transformForFarSide(
      PathPlanner.loadPathGroup(
        "wallConeCubeStation",
        PathConstraints(
          2.5,
          0.75 // Once worked out, at least 3.5
        )
      )
    )

  /** Description of path: Far (near center of field) side path that scores a cone, and then balances on the station */
  val FARCONESTATION: MutableList<PathPlannerTrajectory> =
    AutoUtil.transformForFarSide(
      PathPlanner.loadPathGroup(
        "wallConeStation",
        PathConstraints(
          2.5,
          0.75 // Once worked out, at least 3.5
        )
      )
    )

  /** Description of path: Far (near center of field) side path that scores a cone, then a cube */
  val FARCONECUBE: MutableList<PathPlannerTrajectory> =
    AutoUtil.transformForFarSide(
      PathPlanner.loadPathGroup(
        "wallConeCube",
        PathConstraints(
          2.5,
          0.75 // Once worked out, at least 3.5
        )
      )
    )

  /** Description of path: Far (near center of field) side path that scores a cone, and aligns up to a game piece for teleop */
  val FARCONE: MutableList<PathPlannerTrajectory> =
    AutoUtil.transformForFarSide(
      PathPlanner.loadPathGroup(
        "wallCone",
        PathConstraints(
          2.5,
          0.75 // Once worked out, at least 3.5
        )
      )
    )
}
