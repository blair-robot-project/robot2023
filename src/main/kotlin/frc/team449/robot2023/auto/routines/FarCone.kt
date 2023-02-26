package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.Paths
import frc.team449.robot2023.constants.arm.ArmConstants
import frc.team449.robot2023.subsystems.arm.control.ArmFollower

class FarCone(
  private val robot: Robot
) {

  fun routine(): Command {
    val routine =
      HolonomicRoutine(
        drive = robot.drive,
        eventMap = hashMapOf(
          "dropCone" to ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.HIGH) }.andThen(InstantCommand({ robot.intake.pistonRev() })),
          "stowAndGround" to SequentialCommandGroup(
            ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.STOW) },
            ArmFollower(robot.arm) { robot.arm.chooseTraj(ArmConstants.CONE) }
          )
        )
      )

    return routine.fullAuto(Paths.FARCONE)
  }
}