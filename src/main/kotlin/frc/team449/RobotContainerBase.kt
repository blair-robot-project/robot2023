package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.control.OI
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.system.SimBattery

abstract class RobotContainerBase {

  val field = Field2d()

  abstract val powerDistribution: PowerDistribution

  abstract val drive: HolonomicDrive

  abstract val oi: OI

//  abstract val driveSim: DriveSubsystem.SimController? // TODO SIM

  private val simBattery: SimBattery = SimBattery()

  open fun robotInit() {
//    if (driveSim != null) {
//      simBattery.addCurrentDrawer(driveSim!!::getCurrentDraw)
//    }
  }

  open fun robotPeriodic() {}

  open fun teleopInit() {}

  open fun teleopPeriodic() {}

  open fun autonomousInit() {}

  open fun simulationInit() {}

  open fun simulationPeriodic() {
    simBattery.update()
  }

  open fun disabledInit() {
  }
}
