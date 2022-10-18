package frc.team449.control.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.robot2022.RobotContainer2022
import frc.team449.robot2022.auto.Example

class AutoChooser(robot: RobotContainer2022) : SendableChooser<AutoRoutine>() {
  init {
    // Add options here
    this.addOption("example", Example(robot = robot).routine())
  }
}
