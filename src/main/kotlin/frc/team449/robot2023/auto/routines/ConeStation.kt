package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.commands.AutoBalance
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class ConeStation(
  private val robot: Robot,
  private val farSide: Boolean
): RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf(
        "dropCone" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }
          .andThen(WaitCommand(3.0))
          .andThen(InstantCommand({ robot.intake.pistonRev() })),
        "stowArm" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
        "balanceStation" to AutoBalance(
          PIDController(1.0, 0.0, 0.0),
          robot.drive,
          0.5
        )
      )
    )

  override val trajectory: MutableList<PathPlannerTrajectory>
    get() = if (farSide) {
      Paths.FARCONESTATION
    } else {
      Paths.WALLCONESTATION
    }
}
