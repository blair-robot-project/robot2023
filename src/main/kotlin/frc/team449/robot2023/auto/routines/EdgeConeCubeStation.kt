package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.constants.subsystem.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCubeStation(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to AutoUtil.dropCone(robot),
        "stowAndGround" to SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
        ),
        "stowAndHigh" to SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
        ),
        "dropCube" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.andThen(InstantCommand({ robot.endEffector.pistonRev() })),
        "stowArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
        "balanceStation" to AutoBalance.create(robot.drive)
      )
    )

  override val trajectory =
    if (position == PositionChooser.POSITIONS.FAR) {
      Paths.FARCONECUBESTATION
    } else {
      Paths.WALLCONECUBESTATION
    }
}
