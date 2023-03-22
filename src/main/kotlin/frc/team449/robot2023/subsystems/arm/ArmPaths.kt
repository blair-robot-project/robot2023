package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {

  // scoring trajs
//  var MID_STOW = ArmTrajectory("Mid_Stow.json")
//  var STOW_MID = ArmTrajectory("Stow_Mid.json")
//  var HIGH_STOW = ArmTrajectory("High_Stow.json")
//  var STOW_HIGH = ArmTrajectory("Stow_High.json")
//  var GROUND_STOW = ArmTrajectory("Ground_Stow.json")
//  var STOW_GROUND = ArmTrajectory("Stow_Ground.json")

  var stowSingle = ArmTrajectory("stowSingle.json")
  var singleStow = ArmTrajectory("singleStow.json")

  // todo: get position for double substation and create trajectory. put trajectory into stowDouble and doubleStow

  var stowDouble = ArmTrajectory("stowDouble.json")
  var doubleStow = ArmTrajectory("doubleStow.json")

  var stowCone = ArmTrajectory("stowCone.json")
  var coneStow = ArmTrajectory("coneStow.json")
  var stowCube = ArmTrajectory("stowCube.json")
  var cubeStow = ArmTrajectory("cubeStow.json")

  var stowMid = ArmTrajectory("stowMid.json")
  var midStow = ArmTrajectory("midStow.json")

  var stowHigh = ArmTrajectory("stowHigh.json")
  var highStow = ArmTrajectory("highStow.json")

  /** Auto Specific trajectories */

  var coneHigh = ArmTrajectory("coneHigh.json")
  var highCone = ArmTrajectory("highCone.json")

  var cubeHigh = ArmTrajectory("cubeHigh.json")
  var highCube = ArmTrajectory("highCube.json")
  fun parseTrajectories() {
    listOf(
      stowSingle,
      singleStow,
      stowDouble,
      doubleStow,
      stowCone,
      coneStow,
      stowCube,
      cubeStow,
      stowMid,
      midStow,
      stowHigh,
      highStow,
      coneHigh,
      highCone,
      cubeHigh,
      highCube
    ).forEach {
      it.parse()
    }
  }
}
