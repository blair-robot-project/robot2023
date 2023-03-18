package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {

  // scoring trajs
  var MID_STOW = ArmTrajectory("Mid_Stow.json")
  var STOW_MID = ArmTrajectory("Stow_Mid.json")
  var HIGH_STOW = ArmTrajectory("High_Stow.json")
  var STOW_HIGH = ArmTrajectory("Stow_High.json")
  var GROUND_STOW = ArmTrajectory("Ground_Stow.json")
  var STOW_GROUND = ArmTrajectory("Stow_Ground.json")
  fun parseTrajectories() {
    MID_STOW.parse()
    STOW_MID.parse()
    HIGH_STOW.parse()
    STOW_HIGH.parse()
    GROUND_STOW.parse()
    STOW_GROUND.parse()
  }
}
