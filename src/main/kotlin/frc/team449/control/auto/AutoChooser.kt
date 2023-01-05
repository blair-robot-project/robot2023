package frc.team449.control.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.robot2022.Robot
import frc.team449.robot2022.auto.Example
import frc.team449.robot2022.auto.Foo

class AutoChooser(robot: Robot) : SendableChooser<AutoRoutine>() {
  init {
    // Add options here
    this.setDefaultOption("example", Example(robot = robot).routine())
    this.addOption("foo", Foo(robot = robot).routine())
  }
}
