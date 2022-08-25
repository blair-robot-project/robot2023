package frc.team449.robot2022.drive

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units

object DriveConstants {

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 5
  const val DRIVE_MOTOR_FR = 1
  const val DRIVE_MOTOR_BL = 8
  const val DRIVE_MOTOR_BR = 4
  const val TURN_MOTOR_FL = 6
  const val TURN_MOTOR_FR = 2
  const val TURN_MOTOR_BL = 7
  const val TURN_MOTOR_BR = 3

  /** Absolute (Duty Cycle) encoder channels */
  const val TURN_ENC_CHAN_FL = 0
  const val TURN_ENC_CHAN_FR = 3
  const val TURN_ENC_CHAN_BL = 1
  const val TURN_ENC_CHAN_BR = 2

  /** Offsets for the absolute encoders in rotations */
  const val TURN_ENC_OFFSET_FL = 0.955
  const val TURN_ENC_OFFSET_FR = 0.286
  const val TURN_ENC_OFFSET_BL = 0.369
  const val TURN_ENC_OFFSET_BR = 0.362

  /** FF values for turning each module */
  const val TURN_KS = 0.039835
  const val TURN_KV = 0.078855
  const val TURN_KA = 0.027521

  /** PID gains for turning each module*/
  const val TURN_KP = 0.25
  const val TURN_KI = 0.0
  const val TURN_KD = 0.001

  /** FF values for driving each module */
  const val DRIVE_KS = 0.17227
  const val DRIVE_KV = 2.7582
  const val DRIVE_KA = 0.2595

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.09
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  const val DRIVE_GEARING = 5.86
  const val DRIVE_UPR = 0.3021211527151539
  const val TURN_UPR = 2 * Math.PI * 1

  /** Location of the front left module */
  val FRONT_LEFT_LOC = Translation2d(
    Units.inchesToMeters(24.0) / 2,
    Units.inchesToMeters(24.0) / 2
  )

  const val MAX_LINEAR_SPEED = 2.0
  const val MAX_ROT_SPEED = 2.0
}
