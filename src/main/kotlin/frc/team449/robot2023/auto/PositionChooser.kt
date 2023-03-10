package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

class PositionChooser : SendableChooser<PositionChooser.POSITIONS>() {
  enum class POSITIONS {
    FARCONE,
    WALLCONE,
    FARCUBE,
    WALLCUBE,
    CENTER
  }

  init {
    this.setDefaultOption("Cone Wall side", POSITIONS.WALLCONE)
    this.addOption("Cone Far side", POSITIONS.FARCONE)
    this.addOption("Cube Wall side", POSITIONS.WALLCUBE)
    this.addOption("Cube Far side", POSITIONS.FARCUBE)
    this.addOption("Center (Only drop piece and 1 Piece Balance)", POSITIONS.CENTER)
  }
}
