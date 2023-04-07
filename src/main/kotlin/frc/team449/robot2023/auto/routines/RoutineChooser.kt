package frc.team449.robot2023.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot
import frc.team449.robot2023.auto.PositionChooser

class RoutineChooser(private val robot: Robot, position: PositionChooser) : SendableChooser<RoutineStructure>() {

  init {
    updateOptions(position.selected)
  }

  // TODO: Update all routines to make use of on the move event following :D
  fun updateOptions(position: PositionChooser.Positions) {
    /** Add auto options here */
    this.setDefaultOption("Drop Piece", DropCone(robot))

    this.addOption(
      "1.5 Piece (Edges only)",
      when (position) {
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeCone(robot, position)
        }

        PositionChooser.Positions.FARCUBE, PositionChooser.Positions.WALLCUBE -> {
          EdgeCube(robot, position)
        }

        else -> {
          DropCone(robot)
        }
      }
    )

    this.addOption(
      "2.5 Piece (Edges only)",
      when (position) {
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeConeCube(robot, position)
        }

        PositionChooser.Positions.FARCUBE, PositionChooser.Positions.WALLCUBE -> {
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
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeConeStation(robot, position)
        }

        PositionChooser.Positions.FARCUBE, PositionChooser.Positions.WALLCUBE -> {
          EdgeCubeStation(robot, position)
        }

        else -> {
          CenterCubeStation(robot)
        }
      }
    )

    this.addOption(
      "2.5 Piece and Balance (Edges only)",
      when (position) {
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeConeCubeStation(robot, position)
        }

        PositionChooser.Positions.FARCUBE, PositionChooser.Positions.WALLCUBE -> {
          EdgeCubeConeStation(robot, position)
        }

        else -> {
          DropCone(robot)
        }
      }
    )

    this.addOption(
      "High Link (Edges only)",
      when (position) {
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeConeCubeCone(robot, position)
        }

        PositionChooser.Positions.FARCUBE, PositionChooser.Positions.WALLCUBE -> {
          EdgeCubeConeCone(robot, position)
        }

        else -> {
          DropCone(robot)
        }
      }
    )

    this.addOption(
      "3 Piece (Starting Edge Cone Only)",
      when (position) {
        PositionChooser.Positions.FARCONE, PositionChooser.Positions.WALLCONE -> {
          EdgeConeCubeCube(robot, position)
        }

        else -> {
          DropCone(robot)
        }
      }
    )
  }
}
