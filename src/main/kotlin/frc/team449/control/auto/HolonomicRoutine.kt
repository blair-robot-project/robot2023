package frc.team449.control.auto

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.BaseAutoBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.robot2023.auto.AutoConstants
import io.github.oblarg.oblog.annotations.Config
import java.util.function.BooleanSupplier
import kotlin.math.PI

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
  private val timeout: Double = 1.0
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

    commands.addCommands(resetPose(transformForAlliance(pathGroup, 0) { DriverStation.getAlliance() == DriverStation.Alliance.Red }))

    for ((index, traj) in pathGroup.withIndex()) {
      commands.addCommands(stopEventGroup(pathGroup[index].startStopEvent))
      val wantedTrajectory = if (index == 0) traj else transformForAlliance(pathGroup, index) { DriverStation.getAlliance() == DriverStation.Alliance.Red }
      commands.addCommands(
        followPathWithEvents(
          wantedTrajectory
        )
      )
    }

    commands.addCommands(stopEventGroup(pathGroup[pathGroup.size - 1].endStopEvent))

    return commands
  }

  private fun transformForAlliance(pathGroup: MutableList<PathPlannerTrajectory>, index: Int, isRed: BooleanSupplier): PathPlannerTrajectory {
    println("HEY THIS IS THE THING" + isRed.asBoolean)
    if (isRed.asBoolean) {
        pathGroup[index] = PathPlannerTrajectory.transformTrajectoryForAlliance(
          pathGroup[index],
          DriverStation.getAlliance()
        )

        for (s in pathGroup[index].states) {
          s as PathPlannerTrajectory.PathPlannerState
          s.poseMeters = Pose2d(16.4846 - s.poseMeters.x, 8.02 - s.poseMeters.y, (s.poseMeters.rotation.plus(Rotation2d(PI))))
          s.holonomicRotation = s.holonomicRotation.plus(Rotation2d(PI))
        }
    }
    return pathGroup[index]
  }
}
