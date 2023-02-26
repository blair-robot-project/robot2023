package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.Example
import frc.team449.robot2023.auto.routines.FarCone
import frc.team449.robot2023.auto.routines.FarConeCube
import frc.team449.robot2023.auto.routines.FarConeCubeStation
import frc.team449.robot2023.auto.routines.FarConeStation
import frc.team449.robot2023.auto.routines.WallCone
import frc.team449.robot2023.auto.routines.WallConeCube
import frc.team449.robot2023.auto.routines.WallConeCubeStation
import frc.team449.robot2023.auto.routines.WallConeStation

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
