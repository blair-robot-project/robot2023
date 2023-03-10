package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeCubeConeStation(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.dropPiece(robot),
        "deployIntake" to AutoUtil.deployGroundIntake(robot),
        "runIntake" to InstantCommand(robot.groundIntake::runIntake),
        "stowArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
        "stopIntake" to AutoUtil.retractGroundIntake(robot),
        "handoff" to robot.groundIntake.handoff(),
        "highArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) },
        "dropCube" to AutoUtil.dropPiece(robot),
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory =
    if (position == PositionChooser.POSITIONS.FARCUBE) {
      Paths.FAR.CUBECONESTATION
    } else {
      Paths.WALL.CUBECONESTATION
    }
}
