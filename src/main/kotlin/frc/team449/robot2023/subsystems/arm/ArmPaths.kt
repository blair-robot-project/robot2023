package frc.team449.robot2023.subsystems.arm

import frc.team449.robot2023.subsystems.arm.control.ArmTrajectory

object ArmPaths {
  var STATION_STOW = ArmTrajectory(
    "station_stow.json"
  )

  var STOW_STATION = ArmTrajectory(
    "stow_station.json"
  )
  var STOW_MID1 = ArmTrajectory(
    "stow_mid1.json"
  )
  var ZERO_STOW = ArmTrajectory(
    "zero_stow.json"
  )
}
