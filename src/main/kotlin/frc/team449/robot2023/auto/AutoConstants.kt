package frc.team449.robot2023.auto

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 1.25
  const val DEFAULT_Y_KP = 1.25
  const val DEFAULT_ROTATION_KP = 0.75

  /** Auto Balance PD Gains */
  const val AUTO_BAL_KP = 0.55
  const val AUTO_BAL_KD = .0325
  const val ADJUST_SPEED = 1.5 // m/s
  const val MAX_ROT_VEL = 100.0 // rad/s
}
