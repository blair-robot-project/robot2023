package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {
  var ZERO_STOW = ArmTrajectory("zero_stow.json")
  var STOW_ZERO = ArmTrajectory("stow_zero.json")
  var EXTEND_STOW = ArmTrajectory("extend_stow_slow.json")
  var STOW_EXTEND = ArmTrajectory("stow_extend_slow.json")
}
