package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.BaseAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.auto.AutoConstants
import frc.team449.robot2023.auto.AutoUtil
import io.github.oblarg.oblog.annotations.Config
import kotlin.collections.HashMap

/**
 * @param xController The PID controller used to correct X translation error when following a trajectory
 * @param yController The PID controller used to correct Y translation error when following a trajectory
 * @param thetaController The PID controller used to correct rotational  error when following a trajectory
 * @param drive The holonomic drive subsystem to be used when following a trajectory
 * @param eventMap A hash map of event marker names paired with the command you want to run that cannot require drive
 * @param translationTol Allowed error in meters when following a trajectory
 * @param thetaTol Allowed error in radians when following a trajectory
 * @param resetPosition Whether to reset your position to the initial pose for all parts of the trajectory
 * @param timeout Maximum time in seconds for the path follower to correct itself after EACH trajectory is done
 */
class HolonomicRoutine(
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val drive: HolonomicDrive,
  eventMap: HashMap<String, Command>,
  private val translationTol: Double = 0.05,
  private val thetaTol: Double = 0.05,
  private val resetPosition: Boolean = false,
  private val timeout: Double = 0.35
) : BaseAutoBuilder(drive::pose, eventMap, DrivetrainType.HOLONOMIC) {

  /** What command you want to use to follow a given trajectory */
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
      timeout,
      resetPose = resetPosition
    )
  }

  // TODO: If AprilTag is in sight at starting pos: either delete resetPose from fullAuto, or make this method do nothing
  override fun resetPose(trajectory: PathPlannerTrajectory): CommandBase {
    return InstantCommand({ drive.pose = trajectory.initialHolonomicPose })
  }

  override fun fullAuto(pathGroup: MutableList<PathPlannerTrajectory>): CommandBase {
    val commands = SequentialCommandGroup()

    val correctedTrajGroup = AutoUtil.transformForAlliance(pathGroup) { DriverStation.getAlliance() == DriverStation.Alliance.Red }

    commands.addCommands(resetPose(correctedTrajGroup[0]))

    for ((index, _) in pathGroup.withIndex()) {
      commands.addCommands(stopEventGroup(correctedTrajGroup[index].startStopEvent))
      commands.addCommands(
        followPathWithEvents(
          correctedTrajGroup[index]
        )
      )
    }

    commands.addCommands(stopEventGroup(correctedTrajGroup[correctedTrajGroup.size - 1].endStopEvent))

    return commands
  }
}
