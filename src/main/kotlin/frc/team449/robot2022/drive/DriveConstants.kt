package frc.team449.robot2022.drive

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units

object DriveConstants {

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 1
  const val DRIVE_MOTOR_FR = 2
  const val DRIVE_MOTOR_BL = 3
  const val DRIVE_MOTOR_BR = 4

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.16475
  const val DRIVE_KV = 2.0909
  const val DRIVE_KA = 0.29862

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.35
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  const val DRIVE_GEARING = 1 / 8.0
  val DRIVE_UPR = Math.PI * Units.inchesToMeters(6.0)
  const val MAX_LINEAR_SPEED = 1.5 // 1.5
  const val MAX_ROT_SPEED = .1 // m'/s
  const val MAX_ATTAINABLE_MK4I_SPEED = (12 - DRIVE_KS) / DRIVE_KV
  const val MAX_ACCEL = 5.0

  /** Controller Configurations */
  const val RATE_LIMIT = 1.5
  const val TRANSLATION_DEADBAND = .125
  const val ROTATION_DEADBAND = .125

  val CAM_NAME = "limelight"
  val ROBOT_TO_CAM = Transform3d()
  val TAG_LAYOUT = AprilTagFieldLayout(listOf(AprilTag(0, Pose3d())), 16.4846, 8.1026)

  val INITAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-180.0))

  val WHEELBASE = Units.inchesToMeters(21.426)
  val TRACKWIDTH = Units.inchesToMeters(21.000)
}
