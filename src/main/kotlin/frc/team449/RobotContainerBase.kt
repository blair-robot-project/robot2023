package frc.team449

import frc.team449.system.SimBattery

abstract class RobotContainerBase {

//  val field = Field2d()
//
//  abstract val autoChooser: SendableChooser<AutoRoutine>
//
//  abstract val powerDistribution: PowerDistribution
//
//  abstract val drive: HolonomicDrive
//
//  abstract val oi: OI

//  abstract val driveSim: DriveSubsystem.SimController? // TODO SIM

  private val simBattery: SimBattery = SimBattery()

  open fun robotInit() {}

  open fun robotPeriodic() {}

  open fun teleopInit() {}

  open fun teleopPeriodic() {
//    drive.set(oi.get())
//    drive.periodic()
  }

  open fun autonomousInit() {}

  open fun autonomousPeriodic() {}

  open fun simulationInit() {}

  open fun simulationPeriodic() {
    simBattery.update()
  }
  open fun disabledInit() {
  }
}
