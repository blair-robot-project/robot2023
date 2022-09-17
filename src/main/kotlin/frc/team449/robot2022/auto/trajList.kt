package frc.team449.robot2022.auto

import com.pathplanner.lib.PathPlanner

object trajList {

  val testAutoPath =
    PathPlanner.loadPath(
      "example",
      AutoConstants.MAX_VEL,
      AutoConstants.MAX_ACC
    )
}
