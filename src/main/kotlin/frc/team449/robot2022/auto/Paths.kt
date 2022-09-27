package frc.team449.robot2022.auto

import com.pathplanner.lib.PathPlanner

object Paths {

  val TEST =
    PathPlanner.loadPath(
      "Simple",
      AutoConstants.MAX_VEL,
      AutoConstants.MAX_ACC
    )
  val FIVE_BALL =
    PathPlanner.loadPath(
      "wishes",
      AutoConstants.MAX_VEL,
      AutoConstants.MAX_ACC
    )
}
