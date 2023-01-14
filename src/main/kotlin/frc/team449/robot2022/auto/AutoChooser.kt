package frc.team449.robot2022.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.AutoRoutine
import frc.team449.robot2022.Robot

class AutoChooser(robot: Robot) : SendableChooser<AutoRoutine>() {
  init {
    // Add options here
    this.setDefaultOption(Example(robot = robot).routine().name, Example(robot = robot).routine())
  }
}
