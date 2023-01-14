package frc.team449.robot2022.constants

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import frc.team449.system.VisionCamera

/** Constants that have anything to do with vision */
object VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(0, Pose3d())
    ),
    16.4846,
    8.1026
  )
  /** List of cameras that we want to use*/
  val CAMERAS = listOf(
    VisionCamera (
      "limelight",
      Transform3d(),
      TAG_LAYOUT
    )
  )
}