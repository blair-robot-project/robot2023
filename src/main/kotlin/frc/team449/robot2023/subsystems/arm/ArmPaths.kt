package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {
  var ZERO_STOW = ArmTrajectory("zero_stow.json")
  var STOW_ZERO = ArmTrajectory("stow_zero.json")
}