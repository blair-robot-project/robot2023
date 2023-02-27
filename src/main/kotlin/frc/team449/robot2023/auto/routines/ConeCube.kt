package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class ConeCube(
  private val robot: Robot,
  private val farSide: Boolean
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
          .andThen(WaitCommand(3.0))
          .andThen(InstantCommand({ robot.intake.pistonRev() })),
        "stowAndGround" to SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
        ),
        "stowAndHigh" to SequentialCommandGroup(
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
          ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
        ),
        "dropCube" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.andThen(InstantCommand({ robot.intake.pistonRev() })),
        "stowArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) }
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory>
    get() = if (farSide) {
      Paths.FARCONECUBE
    } else {
      Paths.WALLCONECUBE
    }
}
