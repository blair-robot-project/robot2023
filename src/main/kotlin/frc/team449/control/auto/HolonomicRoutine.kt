package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.BaseAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2022.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config

/**
 * @param xController The PID controller used to correct X translation error when following a trajectory
 * @param yController The PID controller used to correct Y translation error when following a trajectory
 * @param thetaController The PID controller used to correct rotational  error when following a trajectory
 * @param drive The holonomic drive subsystem to be used when following a trajectory
 * @param eventMap A hash map of event marker names paired with the command you want to run that cannot require drive
 * @param driveEventMap A hash map of the stop point number (the first stop point is the start of the path) paired with the command you want to run that may require drive
 * @param translationTol Allowed error in meters when following a trajectory
 * @param thetaTol Allowed error in radians when following a trajectory
 * @param resetPosition Whether to reset your position to the inital pose in the first trajectory
 * @param timeout Maximum time in seconds for the path follower to correct itself after EACH trajectory is done
 */
class HolonomicRoutine(
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val drive: HolonomicDrive,
  eventMap: HashMap<String, Command>,
  private val driveEventMap: HashMap<Int, Command>,
  private val translationTol: Double = 0.05,
  private val thetaTol: Double = 0.05,
  private val resetPosition: Boolean = false,
  private val timeout: Double = 2.0
) : BaseAutoBuilder(drive::pose, eventMap, DrivetrainType.HOLONOMIC) {

  override fun followPath(trajectory: PathPlannerTrajectory): CommandBase {
    return HolonomicFollower(
      drive,
      trajectory,
      xController,
      yController,
      thetaController,
      Pose2d(
        translationTol,
        translationTol,
        Rotation2d(thetaTol)
      ),
      timeout
    )
  }

  fun constructRoutine(pathGroup: MutableList<PathPlannerTrajectory>): CommandBase {
    val command = SequentialCommandGroup()

    if (resetPosition) {
      // Assume the initial position of the robot is the first pose in the path group
      drive.pose = pathGroup[0].initialHolonomicPose
    }

    // For every path in the path group, stop at stop points, run a drive-requiring command, and then run the trajectory segment
    for (index in 0 until pathGroup.size) {
      command.addCommands(stopEventGroup(pathGroup[index].startStopEvent))
      if (driveEventMap.containsKey(index)) {
        command.addCommands(driveEventMap.getValue(index))
      }
      command.addCommands(followPathWithEvents(pathGroup[index]))
    }

    // Stop the last trajectory following command
    command.addCommands(stopEventGroup(pathGroup[pathGroup.size - 1].endStopEvent))

    return command
  }
}
