package frc.team449.robot2023.constants.drives

import edu.wpi.first.math.util.Units

object SwerveConstants {
  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 1
  const val DRIVE_MOTOR_FR = 2
  const val DRIVE_MOTOR_BL = 3
  const val DRIVE_MOTOR_BR = 4
  const val TURN_MOTOR_FL = 11
  const val TURN_MOTOR_FR = 12
  const val TURN_MOTOR_BL = 13
  const val TURN_MOTOR_BR = 14

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 0
  const val TURN_ENC_CHAN_FR = 0
  const val TURN_ENC_CHAN_BL = 0
  const val TURN_ENC_CHAN_BR = 0

  /** Offsets for the absolute encoders in rotations */
  const val TURN_ENC_OFFSET_FL = 0.035288
  const val TURN_ENC_OFFSET_FR = 0.112509
  const val TURN_ENC_OFFSET_BL = 0.870836 - 0.5
  const val TURN_ENC_OFFSET_BR = 0.963748 - 0.5

  /** PID gains for turning each module */
  const val TURN_KP = 0.75
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.14947
  const val DRIVE_KV = 2.6075
  const val DRIVE_KA = 0.20546

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.4
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  const val DRIVE_GEARING = 1 / 6.75
  const val DRIVE_UPR = 0.31818905832
  const val TURN_UPR = 2 * Math.PI
  const val MAX_ATTAINABLE_MK4I_SPEED = 4.267

  /** Wheelbase = wheel-to-wheel distance from the side of the robot */
  /** Trackwidth = wheel-to-wheel distance from the front/back of the robot */
  val WHEELBASE = Units.inchesToMeters(32.0)
  val TRACKWIDTH = Units.inchesToMeters(27.75)
}
