package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

class AutoChooser(robot: Robot) : SendableChooser<Command>() {
  init {
    /** Add auto options here */
    this.setDefaultOption("testing", Example(robot = robot).routine())

    this.addOption("Far Cone", FarCone(robot).routine())
    this.addOption("Far Cone and Cube", FarConeCube(robot).routine())
    this.addOption("Far Cone and Balance", FarConeStation(robot).routine())
    this.addOption("Far Cone, Cube, and Balance", FarConeCubeStation(robot).routine())
    this.addOption("Wall Cone", WallCone(robot).routine())
    this.addOption("Wall Cone and Cube", WallConeCube(robot).routine())
    this.addOption("Wall Cone and Balance", WallConeStation(robot).routine())
    this.addOption("Wall Cone, Cube, and Balance", WallConeCubeStation(robot).routine())
  }
}
