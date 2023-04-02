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

  val stowSingle = ArmTrajectory("stowSingle.json")
  val singleStow = ArmTrajectory("singleStow.json")

  // todo: get position for double substation and create trajectory. put trajectory into stowDouble and doubleStow

  val stowDouble = ArmTrajectory("stowDouble.json")
  val doubleStow = ArmTrajectory("doubleStow.json")

  val stowCone = ArmTrajectory("stowCone.json")
  val coneStow = ArmTrajectory("coneStow.json")
  val stowCube = ArmTrajectory("stowCube.json")
  val cubeStow = ArmTrajectory("cubeStow.json")

  val stowMid = ArmTrajectory("stowMid.json")
  val midStow = ArmTrajectory("midStow.json")

  val stowHigh = ArmTrajectory("stowHigh.json")
  val highStow = ArmTrajectory("highStow.json")

  val highMid = ArmTrajectory("highMid.json")
  val midHigh = ArmTrajectory("midHigh.json")

  val cubeCone = ArmTrajectory("cubeCone.json")
  val coneCube = ArmTrajectory("coneCube.json")

  /** Auto Specific trajectories */

  val coneHigh = ArmTrajectory("coneHigh.json")
  val highCone = ArmTrajectory("highCone.json")

  val cubeHigh = ArmTrajectory("cubeHigh.json")
  val highCube = ArmTrajectory("highCube.json")
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
      highCube,
      midHigh,
      highMid,
      cubeCone,
      coneCube
    ).forEach {
      it.parse()
    }
  }
}
