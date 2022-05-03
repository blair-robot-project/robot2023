package frc.team449.robot2022.drive

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder

object DriveConstants {
  /** Controller constants */
  const val DRIVE_CONTROLLER_PORT = 0
  const val DRIVE_TURNING_DEADBAND = .1
  /** Drive motor ports */
  const val DRIVE_MOTOR_L = 2
  const val DRIVE_MOTOR_L1 = 4
  const val DRIVE_MOTOR_L2 = 3
  const val DRIVE_MOTOR_R = 1
  const val DRIVE_MOTOR_R1 = 11
  const val DRIVE_MOTOR_R2 = 7

  /** Drive Sim Constants */
  const val DRIVE_ANGLE_FF_KS = 0.20112
  const val DRIVE_ANGLE_FF_KV = 10.05
  const val DRIVE_ANGLE_FF_KA = 0.505

  /** Drive Constants */
  // Feed Forward
  const val DRIVE_KP_VEL = 2.0
  const val DRIVE_KI_VEL = 0.0
  const val DRIVE_KD_VEL = 0.0
  const val DRIVE_FF_KS = 0.1908
  const val DRIVE_FF_KV = 2.5406
  const val DRIVE_FF_KA = 0.44982
  // Ramping
  const val LINEAR_ACC_LIMIT = 2.0
  const val TURNING_ACC_LIMIT = 120.0
  // Encoders for driving motors
  val DRIVE_ENC_RIGHT = Encoder(4, 5)
  val DRIVE_ENC_LEFT = Encoder(6, 7)
  const val NEO_ENCODER_CPR = 1
  const val DRIVE_EXT_ENC_CPR = 256
  // Drive Characteristics
  val DRIVE_WHEEL_RADIUS = Units.inchesToMeters(2.0)
  const val DRIVE_GEARING = 5.86
  const val DRIVE_UPR = 0.3021211527151539

  const val DRIVE_CURRENT_LIM = 50
  const val DRIVE_ENC_VEL_THRESHOLD = 0.1
  const val MAX_LINEAR_SPEED = 4.0
  const val TRACK_WIDTH = .5 // m
}
