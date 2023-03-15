package frc.team449.robot2023.auto.routines

import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.AutoBalance

class EdgeConeCubeStation(
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
        "dropCube" to AutoUtil.dropCube(robot),
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory =
    if (position == PositionChooser.POSITIONS.FARCONE) {
      Paths.FAR.CONECUBESTATION
    } else {
      Paths.WALL.CONECUBESTATION
    }
}
