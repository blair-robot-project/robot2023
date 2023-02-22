package frc.team449.robot2023.constants

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

object RobotConstants {

  /** Other CAN ID */
  const val PDP_CAN = 1
  const val PCM_MODULE = 0

  /** Controller Configurations */
  const val RATE_LIMIT = 3.5
  const val TRANSLATION_DEADBAND = .125
  const val ROTATION_DEADBAND = .125

  /** Drive configuration */
  const val MAX_LINEAR_SPEED = 1.5 // m/s
  const val MAX_ROT_SPEED = 2.0 // rad/s
  const val MAX_ACCEL = 5.0 // m/s/s
  val INITIAL_POSE = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))
}
