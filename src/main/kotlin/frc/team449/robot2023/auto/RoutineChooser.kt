package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.routines.EdgeCone
import frc.team449.robot2023.auto.routines.EdgeConeCube
import frc.team449.robot2023.auto.routines.EdgeConeCubeStation
import frc.team449.robot2023.auto.routines.EdgeConeStation

class RoutineChooser(private val robot: Robot, position: PositionChooser) : SendableChooser<RoutineStructure>() {
  init {
    /** Add auto options here */
//    this.setDefaultOption("testing", InstantCommand(Example(robot = robot)::routine))
    updateOptions(position.selected)
  }

  fun updateOptions(position: PositionChooser.POSITIONS) {
    this.setDefaultOption("Cone", EdgeCone(robot, position))
    this.addOption("Cone and Cube", EdgeConeCube(robot, position))
    this.addOption("Cone and Balance", EdgeConeStation(robot, position))
    this.addOption("Cone Cube and Balance", EdgeConeCubeStation(robot, position))
  }
}
