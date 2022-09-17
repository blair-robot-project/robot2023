package frc.team449.robot2022.drive

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

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 0
  const val TURN_ENC_CHAN_FR = 3
  const val TURN_ENC_CHAN_BL = 1
  const val TURN_ENC_CHAN_BR = 2

  /** Offsets for the absolute encoders in rotations */
  const val TURN_ENC_OFFSET_FL = 0.955 // Rot
  const val TURN_ENC_OFFSET_FR = 0.286
  const val TURN_ENC_OFFSET_BL = 0.369
  const val TURN_ENC_OFFSET_BR = 0.362

  /** Feed forward values for turning each module */
  const val TURN_KS = 0.039835 // V/rad
  const val TURN_KV = 0.078855
  const val TURN_KA = 0.027521

  /** PID gains for turning each module*/
  const val TURN_KP = 0.7 // V/output[-1, 1]
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.17227 // V/m
  const val DRIVE_KV = 2.7582
  const val DRIVE_KA = 0.2595

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.07 // V/m
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  const val DRIVE_GEARING = 5.86 /** L2 gear ratio from [https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081] */
  const val DRIVE_UPR = 0.3021211527151539 // meters
  const val TURN_UPR = 2 * Math.PI * 1 // rads
  const val MAX_LINEAR_SPEED = 4.0 // m/s
  const val MAX_ROT_SPEED = 2.5 // rad/s
  const val MAX_ATTAINABLE_MK4I_SPEED = 4.267 // m/s
  /** Location of the front left module */
  const val WHEELBASE = 0.47625 // m
  const val TRACKWIDTH = 0.47625 // m
}
