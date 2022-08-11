package frc.team449.robot2022.drive

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

object DriveConstants {

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 0
  const val DRIVE_MOTOR_FR = 1
  const val DRIVE_MOTOR_BL = 2
  const val DRIVE_MOTOR_BR = 3
  const val TURN_MOTOR_FL = 4
  const val TURN_MOTOR_FR = 5
  const val TURN_MOTOR_BL = 6
  const val TURN_MOTOR_BR = 7

  /** External encoders for driving motors */

  /** Absolute (Duty Cycle) encoder channels */
  const val TURN_ENC_CHAN_FL = 0
  const val TURN_ENC_CHAN_FR = 1
  const val TURN_ENC_CHAN_BL = 2
  const val TURN_ENC_CHAN_BR = 3

  /** Offsets for the absolute encoders */
  const val TURN_ENC_OFFSET_FL = 0.0
  const val TURN_ENC_OFFSET_FR = 0.0
  const val TURN_ENC_OFFSET_BL = 0.0
  const val TURN_ENC_OFFSET_BR = 0.0

  val DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2.0)
  const val DRIVE_GEARING = 5.86
  // todo determine this
  const val TURN_GEARING = 1.0
  const val DRIVE_UPR = 0.3021211527151539
  // todo determine this. should be in radians
  const val TURN_UPR = 2 * Math.PI * 1
  const val NEO_ENCODER_CPR = 1
  /** CPR of external encoders on driving motors */
  const val DRIVE_EXT_ENC_CPR = 256

  /** Location of the front left module */
  val FRONT_LEFT_LOC = Translation2d(
    Units.inchesToMeters(25.0) / 2,
    Units.inchesToMeters(25.0) / 2
  )

  const val DRIVE_CURRENT_LIM = 40
  const val DRIVE_ENC_VEL_THRESHOLD = 0.1
  const val MAX_LINEAR_SPEED = 4.0
  const val MAX_ROT_SPEED = 1.0
}
