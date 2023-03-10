package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCube(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.dropCone(robot),
        "stowArm" to AutoUtil.stowAndDeploy(robot),
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "handoff" to robot.groundIntake.handoff(),
        "highArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) },
        "dropCube" to AutoUtil.dropCube(robot),
        "retractArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.POSITIONS.FARCONE) {
      Paths.FAR.CONECUBE
    } else {
      Paths.WALL.CONECUBE
    }
}
