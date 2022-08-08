package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.DriveSubsystem
import frc.team449.control.OI
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.differential.DifferentialDrive
import frc.team449.system.SimBattery

abstract class RobotContainerBase {

  val field = Field2d()

  abstract val autoChooser: SendableChooser<AutoRoutine>

  abstract val powerDistribution: PowerDistribution

  abstract val drive: DifferentialDrive

  abstract val oi: OI

  abstract val driveSim: DriveSubsystem.SimController?

  private val simBattery: SimBattery = SimBattery()

  open fun robotInit() {
    if (driveSim != null) {
      simBattery.addCurrentDrawer(driveSim!!::getCurrentDraw)
    }
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
    if (driveSim != null) {
      driveSim!!.update()
    }
    simBattery.update()
  }
}
