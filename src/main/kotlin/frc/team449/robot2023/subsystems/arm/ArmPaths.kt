package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {
  // intaking trajs
  var INTAKE_STOW = ArmTrajectory("Intake_Stow.json")
  var STOW_INTAKE = ArmTrajectory("Stow_Intake.json")
  var PICKUP_STOW = ArmTrajectory("Pickup_Stow.json")
  var STOW_PICKUP = ArmTrajectory("Stow_Pickup.json")

  // scoring trajs
  var LOW_STOW = ArmTrajectory("Low_Stow.json")
  var STOW_LOW = ArmTrajectory("Stow_Low.json")
  var MID_STOW = ArmTrajectory("Mid_Stow.json")
  var STOW_MID = ArmTrajectory("Stow_Mid.json")
  var HIGH_STOW = ArmTrajectory("High_Stow.json")
  var STOW_HIGH = ArmTrajectory("Stow_High.json")
}
