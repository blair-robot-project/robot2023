package frc.team449.robot2023.auto

object AutoConstants {
  /** PID gains */
  const val DEFAULT_X_KP = 1.8
  const val DEFAULT_Y_KP = 1.8
  const val DEFAULT_ROTATION_KP = 1.4

  /** Auto Balance PD Gains */
  const val AUTO_BAL_KP = 0.6
  const val AUTO_BAL_KD = .0325
  const val ADJUST_SPEED = 2.1 // m/s
  const val MAX_ROT_VEL = 20.0 // deg/s

  const val Y_DISTANCE_BARRIER = 5.47878
  const val FIELD_LENGTH = 16.54
  const val FIELD_WIDTH = 8.02

  // in deg per 20 ms
  const val CONE_DROP_SWEEP_SPEED = 0.365
  const val CONE_DROP_SWEEP_TIME = 0.5

  const val CUBE_DROP_WAIT_BEFORE = 0.075
  const val CUBE_DROP_WAIT_AFTER = 0.2
}
