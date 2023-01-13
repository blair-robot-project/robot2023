package frc.team449.robot2022.auto
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner

object Paths {
  val TEST = PathPlanner.loadPathGroup("testing", PathConstraints(1.0, 0.25))
}
