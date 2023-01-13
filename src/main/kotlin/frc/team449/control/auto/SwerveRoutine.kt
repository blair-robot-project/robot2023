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

class SwerveRoutine(
  @field:Config.PIDController(name = "X PID") var xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Y PID") var yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  @field:Config.PIDController(name = "Rotation PID") var rotController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  private val drive: HolonomicDrive,
  eventMap: HashMap<String, Command>,
  private val driveEventMap: HashMap<Int, Command>,
  private val translationTol: Double = 0.05,
  private val angleTol: Double = 0.05,
  private val resetPosition: Boolean = false,
  private val timeout: Double = 2.5
) : BaseAutoBuilder(drive::pose, eventMap, DrivetrainType.HOLONOMIC) {

  override fun followPath(trajectory: PathPlannerTrajectory): CommandBase {
    return HolonomicFollower(
      drive,
      trajectory,
      xController,
      yController,
      rotController,
      Pose2d(
        translationTol,
        translationTol,
        Rotation2d(angleTol)
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

    for (index in 0 until pathGroup.size) {
      command.addCommands(stopEventGroup(pathGroup[index].startStopEvent))
      if (driveEventMap.containsKey(index)) {
        command.addCommands(driveEventMap.getValue(index))
      }
      command.addCommands(followPathWithEvents(pathGroup[index]))
    }

    command.addCommands(stopEventGroup(pathGroup[pathGroup.size - 1].endStopEvent))

    return command
  }
}
