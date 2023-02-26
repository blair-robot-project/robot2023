package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.Cone
import frc.team449.robot2023.auto.routines.ConeCube
import frc.team449.robot2023.auto.routines.ConeCubeStation
import frc.team449.robot2023.auto.routines.ConeStation
import frc.team449.robot2023.auto.routines.Example

class AutoChooser(private val robot: Robot) : SendableChooser<Command>() {
  init {
    updateOptions()
  }

  fun updateOptions() {
    /** Add auto options here */
    this.setDefaultOption("testing", Example(robot = robot).routine())

    this.addOption("Far Cone", Cone(robot, true).routine())
    this.addOption("Far Cone and Cube", ConeCube(robot, true).routine())
    this.addOption("Far Cone and Balance", ConeStation(robot, true).routine())
    this.addOption("Far Cone Cube and Balance", ConeCubeStation(robot, true).routine())
    this.addOption("Wall Cone", Cone(robot, false).routine())
    this.addOption("Wall Cone and Cube", ConeCube(robot, false).routine())
    this.addOption("Wall Cone and Balance", ConeStation(robot, false).routine())
    this.addOption("Wall Cone Cube and Balance", ConeCubeStation(robot, false).routine())
  }
}
