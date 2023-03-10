package frc.team449.robot2023.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot

class RoutineChooser(private val robot: Robot, position: PositionChooser) : SendableChooser<RoutineStructure>() {

  init {
    updateOptions(position.selected)
  }

  fun updateOptions(position: PositionChooser.POSITIONS) {
    /** Add auto options here */
    this.setDefaultOption("Drop Piece", DropCone(robot))

    this.addOption(
      "1 Piece (Edges only)",
      when (position) {
        PositionChooser.POSITIONS.FARCONE, PositionChooser.POSITIONS.WALLCONE -> {
          EdgeCone(robot, position)
        }
        PositionChooser.POSITIONS.FARCUBE, PositionChooser.POSITIONS.WALLCUBE -> {
          EdgeCube(robot, position)
        }
        else -> {
          DropCone(robot)
        }
      }
    )

    this.addOption(
      "2 Piece (Edges only)",
      when (position) {
        PositionChooser.POSITIONS.FARCONE, PositionChooser.POSITIONS.WALLCONE -> {
          EdgeConeCube(robot, position)
        }
        PositionChooser.POSITIONS.FARCUBE, PositionChooser.POSITIONS.WALLCUBE -> {
          EdgeCubeCone(robot, position)
        }
        else -> {
          DropCone(robot)
        }
      }
    )

    this.addOption(
      "1 Piece and Balance",
      when (position) {
        PositionChooser.POSITIONS.FARCONE, PositionChooser.POSITIONS.WALLCONE -> {
          EdgeConeStation(robot, position)
        }
        PositionChooser.POSITIONS.FARCUBE, PositionChooser.POSITIONS.WALLCUBE -> {
          EdgeCubeStation(robot, position)
        }
        else -> {
          CenterCubeStation(robot)
        }
      }
    )

    this.addOption(
      "2 Piece and Balance (Edges only)",
      when (position) {
        PositionChooser.POSITIONS.FARCONE, PositionChooser.POSITIONS.WALLCONE -> {
          EdgeConeCubeStation(robot, position)
        }
        PositionChooser.POSITIONS.FARCUBE, PositionChooser.POSITIONS.WALLCUBE -> {
          EdgeCubeConeStation(robot, position)
        }
        else -> {
          DropCone(robot)
        }
      }
    )
  }
}
