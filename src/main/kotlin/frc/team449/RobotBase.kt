package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d

abstract class RobotBase {

  val field = Field2d()

  abstract val powerDistribution: PowerDistribution

//  abstract val drive: HolonomicDrive?
//
//  abstract val oi: OI
}
