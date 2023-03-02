package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class PositionChooser : SendableChooser<PositionChooser.POSITIONS>() {
  enum class POSITIONS {
    FAR,
    WALL
  }

  init {
    this.setDefaultOption("Wall side", POSITIONS.WALL)
    this.addOption("Far side", POSITIONS.FAR)
  }
}
