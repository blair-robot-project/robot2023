package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.OI
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.holonomic.HolonomicDrive
import frc.team449.system.SimBattery

abstract class RobotContainerBase {

  val field = Field2d()

  abstract val autoChooser: SendableChooser<AutoRoutine>

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

  open fun teleopPeriodic() {
    drive.set(oi.get())
    drive.periodic()
  }

  open fun autonomousInit() {
  }

  open fun simulationInit() {
  }

  open fun simulationPeriodic() {
//    if (driveSim != null) {
//      driveSim!!.update()
    //    } // TODO Swerve Drive SIM
    simBattery.update()
  }
}
