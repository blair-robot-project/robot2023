package frc.team449.robot2023.constants.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

/** Constants that have anything to do with vision */
object
VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(3, Pose3d())
    ),
    16.4846,
    8.1026
  )

  /** WPILib's AprilTagFieldLayout for the 2023 Charged Up Game */
  private val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
    AprilTagFields.k2023ChargedUp.m_resourceFile
  )

  /** Robot to Camera distance */
  private val robotToCamera = Transform3d(
    Translation3d(Units.inchesToMeters(-10.25), Units.inchesToMeters(-6.0), Units.inchesToMeters(13.35)),
    Rotation3d(0.0, Units.degreesToRadians(12.0), Units.degreesToRadians(180.0))
  )

  var MIN_TARGETS = 2
  var VISION_TRUST = MatBuilder(Nat.N3(), Nat.N1()).fill(.125, .125, .025)
  var ENCODER_TRUST = MatBuilder(Nat.N3(), Nat.N1()).fill(.025, .025, .75)

  /** List of cameras that we want to use */
  val ESTIMATORS: List<PhotonPoseEstimator> = listOf(
    PhotonPoseEstimator(
      TAG_LAYOUT,
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
      PhotonCamera("Spinel"),
      robotToCamera
    )
  )
}
