package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.Cone
import frc.team449.robot2023.auto.routines.ConeCube
import frc.team449.robot2023.auto.routines.ConeCubeStation
import frc.team449.robot2023.auto.routines.ConeStation

class AutoChooser(robot: Robot) : SendableChooser<RoutineStructure>() {
  init {
    /** Add auto options here */
//    this.setDefaultOption("testing", InstantCommand(Example(robot = robot)::routine))

    this.setDefaultOption("Far Cone", Cone(robot, true))
    this.addOption("Far Cone and Cube", ConeCube(robot, true))
    this.addOption("Far Cone and Balance", ConeStation(robot, true))
    this.addOption("Far Cone Cube and Balance", ConeCubeStation(robot, true))
    this.addOption("Wall Cone", Cone(robot, false))
    this.addOption("Wall Cone and Cube", ConeCube(robot, false))
    this.addOption("Wall Cone and Balance", ConeStation(robot, false))
    this.addOption("Wall Cone Cube and Balance", ConeCubeStation(robot, false))
  }
}
