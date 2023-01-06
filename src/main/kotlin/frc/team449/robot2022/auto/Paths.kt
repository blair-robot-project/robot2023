package frc.team449.robot2022.auto
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner

object Paths {
  val TEST = PathPlanner.loadPathGroup("Simple", PathConstraints(1.0, 1.0))
}
