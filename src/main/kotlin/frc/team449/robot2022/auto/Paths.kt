package frc.team449.robot2022.auto

import com.pathplanner.lib.PathPlanner

object Paths {
  val TEST =
    PathPlanner.loadPath(
      "9",
      PathPlanner.getConstraintsFromPath("9")
    )
}
