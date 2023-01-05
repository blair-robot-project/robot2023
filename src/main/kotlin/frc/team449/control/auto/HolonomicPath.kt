package frc.team449.control.auto

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.PIDController
import frc.team449.robot2022.auto.AutoConstants
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Config

/**
 * A tunable holonomic path
 * @param resetPose Whether to reset the robot pose to the first pose in the trajectory
 * @param translationTol Tolerance between the translation of the actual and desired end pose of the trajectory in meters
 * @param angleTol Tolerance between the angle of the actual and desired end pose of the trajectory in radians
 * @param timeout Maximum time to wait for the robot to fix its end pose. Gives an upper bound of the time the trajectory takes
 */
class HolonomicPath(
  private val fileName: String,
  val resetPose: Boolean,
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var rotController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  var translationTol: Double = .005,
  var angleTol: Double = .001,
  var timeout: Double = 1.0
) : Loggable {
  val trajectory: PathPlannerTrajectory = PathPlanner.loadPath(
    fileName,
    0.5,
    0.5
  )

  override fun configureLogName(): String {
    return "$fileName Path"
  }
}
