package frc.team449.robot2023.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

/** Constants that have anything to do with vision */
object VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(
        3,
        Pose3d(
          Translation3d(0.0, Units.inchesToMeters(24.5), Units.inchesToMeters(32.5)), Rotation3d()
        )
      ),
      AprilTag(
        6,
        Pose3d(
          Translation3d(0.0, Units.inchesToMeters(0.0), Units.inchesToMeters(32.5)), Rotation3d()
        )
      )
    ),
    16.4846,
    8.1026
  )

  /** WPILib's AprilTagFieldLayout for the 2023 Charged Up Game */
  private val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
    AprilTagFields.k2023ChargedUp.m_resourceFile
  )

  /** Robot to Camera distance */
  private val robotToCamera = Transform3d()

  /** List of cameras that we want to use */
  val ESTIMATORS = listOf<PhotonPoseEstimator>(
    PhotonPoseEstimator(
      TEST_TAG_LAYOUT,
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
      PhotonCamera("limelight"),
      robotToCamera
    )
  )
}
