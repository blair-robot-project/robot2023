package frc.team449.robot2022.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import frc.team449.system.VisionCamera

/** Constants that have anything to do with vision */
object VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(0, Pose3d())
    ),
    16.4846,
    8.1026
  )

  /** WPILib's AprilTagFieldLayout for the 2023 Charged Up Game */
  private val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(
    AprilTagFields.k2023ChargedUp.name
  )

  /** List of cameras that we want to use*/
  val CAMERAS = listOf(
    VisionCamera(
      "limelight",
      Transform3d(),
      TEST_TAG_LAYOUT
    )
  )
}
