package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {
  // intaking trajs
  var CONE_STOW = ArmTrajectory("Cone_Stow.json")
  var STOW_CONE = ArmTrajectory("Stow_Cone.json")
  var CUBE_STOW = ArmTrajectory("Cube_Stow.json")
  var STOW_CUBE = ArmTrajectory("Stow_Cube.json")

  // scoring trajs
  var LOW_STOW = ArmTrajectory("Low_Stow.json")
  var STOW_LOW = ArmTrajectory("Stow_Low.json")
  var MID_STOW = ArmTrajectory("Mid_Stow.json")
  var STOW_MID = ArmTrajectory("Stow_Mid.json")
  var HIGH_STOW = ArmTrajectory("High_Stow.json")
  var STOW_HIGH = ArmTrajectory("Stow_High.json")
}
