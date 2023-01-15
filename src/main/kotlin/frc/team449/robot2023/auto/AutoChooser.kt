package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2023.Robot

class AutoChooser(robot: Robot) : SendableChooser<Command>() {
  init {
    /** Add auto options here */
    this.setDefaultOption("Example Auto :]", Example(robot = robot).routine())
  }
}
