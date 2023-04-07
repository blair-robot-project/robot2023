package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCube(
  robot: Robot,
  position: PositionChooser.Positions
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.stowDropCone(robot),
        "stowArm" to AutoUtil.deployCube(robot),
        "stopIntake" to AutoUtil.retractAndStow(robot),
        "dropCube" to ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(AutoUtil.dropCube(robot)),
        "stopIntake2" to AutoUtil.retractAndStow(robot),
        "stowCube" to AutoUtil.deployCube(robot),
        "retractCube" to AutoUtil.retractGroundIntake(robot),
        "highCube" to ArmFollower(robot.arm) { ArmPaths.cubeHigh }.andThen(AutoUtil.dropCube(robot))
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory> =
    if (position == PositionChooser.Positions.FARCONE) {
      Paths.FAR.CONECUBE
    } else {
      Paths.WALL.CONECUBE
    }
}
