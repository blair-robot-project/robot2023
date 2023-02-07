package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.control.OI
import frc.team449.control.differential.DifferentialDrive
import frc.team449.robot2023.subsystems.GroundIntake

abstract class RobotBase {

  val field = Field2d()

  abstract val powerDistribution: PowerDistribution

  abstract val drive: DifferentialDrive

  abstract val oi: OI

  abstract val intake: GroundIntake
}
