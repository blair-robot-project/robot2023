package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot

class AutoChooser(robot: Robot) : SendableChooser<RoutineStructure>() {
  init {
    /** Add auto options here */
//    this.setDefaultOption("testing", InstantCommand(Example(robot = robot)::routine))
  }
}
