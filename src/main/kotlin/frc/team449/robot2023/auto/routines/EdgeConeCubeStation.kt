package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.AutoUtil
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.auto.PositionChooser
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.subsystems.arm.ArmPaths
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class EdgeConeCubeStation(
  robot: Robot,
  position: PositionChooser.POSITIONS
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to ArmFollower(robot.arm) { ArmPaths.stowHigh }.andThen(AutoUtil.dropCone(robot)),
        "stowArm" to AutoUtil.stowAndDeployCube(robot),
        "stopIntake" to AutoUtil.holdIntake(robot),
        "highArm" to ArmFollower(robot.arm) { ArmPaths.cubeHigh },
        "dropCube" to InstantCommand(robot.endEffector::intakeReverse).andThen(
          ArmFollower(robot.arm) { ArmPaths.highStow }
        ),
        "balanceStation" to AutoBalance.create(robot.drive),
      )
    )

  override val trajectory =
    if (position == PositionChooser.POSITIONS.FARCONE) {
      Paths.FAR.CONECUBESTATION
    } else {
      Paths.WALL.CONECUBESTATION
    }
}
