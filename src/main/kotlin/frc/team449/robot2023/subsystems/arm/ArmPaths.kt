package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {

  // scoring trajs
  var MID_STOW = ArmTrajectory("Mid_Stow.json")
  var STOW_MID = ArmTrajectory("Stow_Mid.json")
  var HIGH_STOW = ArmTrajectory("High_Stow.json")
  var STOW_HIGH = ArmTrajectory("Stow_High.json")
  var PICKUP_STOW = ArmTrajectory("Pickup_Stow.json")
  var STOW_PICKUP = ArmTrajectory("Stow_Pickup.json")
  fun parseTrajectories() {
    MID_STOW.parse()
    STOW_MID.parse()
    HIGH_STOW.parse()
    STOW_HIGH.parse()
    PICKUP_STOW.parse()
    STOW_PICKUP.parse()
  }
}
